    /**
    * @file main.c
    * @brief [최종 완성본] C와 Python을 연동하는 비동기 제어 시스템의 메인 프로그램.
    * @details
    * 이 프로그램은 아래의 동작들을 수행함
    * 1.  **프로세스 모델**: C가 부모(지휘자), Python이 자식(AI 분석)으로 동작.
    * 2.  **프로세스 간 통신(IPC)**: 두 개의 파이프(pipe)를 사용해 안정적인 양방향 통신 채널을 구축.
    * 3.  **동적 경로 탐색**: C 실행 파일의 위치를 기준으로 Python 스크립트의 절대 경로를 동적으로 계산하여,
    * 어디서 프로그램을 실행하든 경로 문제 없이 Python을 실행 가능.
    * 4.  **비동기 I/O 처리**: 'select()' 시스템 콜을 사용하여 여러 입력 소스(Python의 응답, CAN 메시지 등)를
    * 하나의 스레드에서 효율적으로 동시에 감시하고 처리. ('AI 분석 대기 중 CAN 통신' 요구사항 해결)
    * 5.  **상태 관리(State Management)**: 비동기적으로 도착하는 데이터들(AI 결과, CAN 메시지)을
    * 상태 변수에 저장했다가, 모든 데이터가 준비되었을 때만 최종 제어 로직을 수행.
    *
    * @compile
    * make clean => 
    * make cross => 타켓 보드용 컴파일
    *
    * @run
    * ./run.sh
    * (종료하려면 터미널에서 Ctrl+C를 누르세요.)
    */

    // --- 1. 필수 헤더 파일 포함 ---
    #include <stdio.h>      // 표준 입출력 함수 (printf, perror, FILE*, fprintf, fflush, fgets)
    #include <stdlib.h>     // 표준 라이브러리 함수 (exit, malloc, free)
    #include <unistd.h>     // 유닉스 표준(POSIX) API (pipe, fork, dup2, execvp, read, write, sleep, close, readlink)
    #include <string.h>     // 문자열 처리 함수 (strlen, strcmp, strerror, strrchr)
    #include <sys/wait.h>   // 자식 프로세스의 종료를 기다리는 waitpid 함수
    #include <errno.h>      // 시스템 에러 코드를 담고 있는 errno 변수
    #include <fcntl.h>      // fcntl() 함수 사용 (파일 디스크립터 속성 제어)
    #include <sys/time.h>   // timeval 구조체 사용 (select 타임아웃)
    #include <sys/select.h> // select() 원형
    #include <signal.h>
    #include <time.h>
    #include <math.h>
    #include "cJSON.h"      // cJSON 라이브러리 사용을 위한 헤더
    #include "hardware.h"


    // --- 2. 전역 변수 ---
    static FILE* stream_to_python = NULL;
    static FILE* stream_from_python = NULL;
    static int pipe_from_python_fd = -1;

    static DetectedObject *g_ai_objs = NULL;
    static int g_ai_count = 0;

    // ===== 파이썬 프로세스 재시작을 위한 전역 상태 =====
    static pid_t g_py_pid = -1;                 // ← 실행 중인 파이썬 자식 프로세스의 PID 저장
    static int c_to_python_pipe[2] = {-1, -1};  // ← C → Python 파이프 (부모가 [1]에 씀, 자식이 [0]에서 읽음)
    static int python_to_c_pipe[2] = {-1, -1};  // ← Python → C 파이프 (자식이 [1]에 씀, 부모가 [0]에서 읽음)

    static int  start_python_process(void);     // ← 파이썬 자식 프로세스를 시작(생성)하는 헬퍼 함수 선언
    static void stop_python_process(void);      // ← 파이썬 자식 프로세스를 종료(정리)하는 헬퍼 함수 선언


    //요청할 PID와 완료 조건을 짝지어 목록으로 정의함
    static const CANRequest pids_to_request[] ={
        {PID_ENGINE_SPEED, ENGINE_SPEED_FLAG},
        {PID_VEHICLE_SPEED, VEHICLE_SPEED_FLAG},
        {PID_GEAR_STATE, GEAR_STATE_FLAG},
        {PID_BRAKE_DATA, BRAKE_DATA_FLAG},
        {PID_TIRE_DATA, TIRE_DATA_FLAG}
    };
    //확인할 PID의 갯수
    static const unsigned char num_pids_to_request = sizeof(pids_to_request) / sizeof(pids_to_request[0]);
    //다음에 확인할 PID의 인덱스를 저장할 변수
    static unsigned char next_pid_index = 0;

    //가속도 측정 구조체
    static SpeedMonitor g_spmon = {0};

    /**
    * @brief 현재 차량 상태를 기반으로 미래 경로(x, y)를 계산하여 배열에 채움
    * * @param current_speed_kph 현재 속도 (km/h)
    * @param current_steer_deg 현재 핸들 조향각 (도)
    * @param out_x 결과가 저장될 X좌표 배열 (전방 거리)
    * @param out_y 결과가 저장될 Y좌표 배열 (측면 거리, +:좌, -:우)
    */
    // void calc_future_path(int current_speed_kph, float current_steer_deg,
    //                       double out_x[POS_COUNT], double out_y[POS_COUNT]){
        
    //     //1. 배열 초기화
    //     memset(out_x, 0, sizeof(double) * POS_COUNT);
    //     memset(out_y, 0, sizeof(double) * POS_COUNT);

    //     //2. 물리량 변환
    //     double v_mps = (double)current_speed_kph * KPH_TO_MPS;

    //        //핸들 각도가 너무 작으면(직진) 0으로 (노이즈 제거)
    //        if(fabs(current_steer_deg) < 1.0){
    //         current_steer_deg = 0.0;
    //        }

    //     //3. 직진 or 곡선 판단 및 계산
    //     if(current_steer_deg == 0.0){
    //         //직진일 때
    //         for(int i = 0; i < POS_COUNT; i++){
    //             double time = (i + 1) * PREDICTION_DT;
    //             out_x[i] = v_mps * time; //직진이므로 거리는 속도 * 시간
    //             out_y[i] = 0.0;       //직진이므로 횡방향 상관x
    //         }
    //     }
    //     else{
    //         //곡선일 때
    //         //핸들을 왼쪽으로 꺾으면 + 
    //         double wheel_angle_rad = (current_steer_deg / STEERING_RATIO) * (M_PI / 180.0);

    //         //회전 반경 계산 R = L / tan(delta)
    //         double R = VEHICLE_WHEELBASE / tan(wheel_angle_rad);

    //         //각속도 계산 w = v / R
    //         double omega = v_mps / R;

    //         for(int i = 0; i < POS_COUNT; i++){
    //             double time = (i + 1) * PREDICTION_DT;

    //             //t초동안 회전한 총 각도(theta)
    //             double theta = omega * time;

    //             //원형 궤적 공식 적용
    //             out_x[i] = R * sin(theta);
    //             out_y[i] = R * (1.0 - cos(theta));
    //         }

    //     }

    static int check_collision_risk(const DetectedObject *ai_objs, int ai_count,
                                 const double *path_x, const double *path_y, int path_count) {
    
    if (!ai_objs || ai_count <= 0 || !path_x || !path_y || path_count <= 0) {
        return 0; // 유효한 데이터가 없으면 위험 없음
    }

    const double R_obj = 1.0; // 객체의 충돌 반경 (1.0 미터)
    // hardware.h에 정의된 차량 크기 사용
    const double L_half = VHEICLE_LENGTH / 2.0;
    const double W_half = VHEICLE_WIDTH / 2.0;

    for (int i = 0; i < ai_count; ++i) {
        const DetectedObject *o = &ai_objs[i];
        
        // 1. 객체의 초기 위치 및 속도 (ax, ay를 Vx, Vy로 사용)
        double initial_x = (double)o->x;
        double initial_y = (double)o->y;
        double speed_x = (double)o->ax;
        double speed_y = (double)o->ay;
        
        // 2. 예측 시간 단계별 충돌 검사
        for (int t_idx = 0; t_idx < path_count; t_idx++) {
            
            // 경과 시간 (t)
            double t_sec = (t_idx + 1) * PREDICTION_DT; 
            
            // 2-1. 차량의 예측 궤적 위치 (차량 중심)
            double path_x_center = path_x[t_idx];
            double path_y_center = path_y[t_idx];
            
            // 2-2. 객체의 미래 예측 위치 (등속 직선 운동)
            double obj_x = initial_x + speed_x * t_sec;
            double obj_y = initial_y + speed_y * t_sec;
            
            // 3. 충돌 검사 로직 (AABB vs. Circle)

            // 객체 중심(obj_x, obj_y)이 차량 AABB에 가장 가까운 지점 찾기
            // AABB 범위: X: [path_x_center - L_half, path_x_center + L_half]
            //            Y: [path_y_center - W_half, path_y_center + W_half]
            
            // X축에서 가장 가까운 AABB 경계 지점
            double closest_x = fmax(path_x_center - L_half, fmin(obj_x, path_x_center + L_half));
            // Y축에서 가장 가까운 AABB 경계 지점
            double closest_y = fmax(path_y_center - W_half, fmin(obj_y, path_y_center + W_half));
            
            // 객체 중심과 AABB 경계 지점 간의 거리 제곱 계산
            double dx = obj_x - closest_x;
            double dy = obj_y - closest_y;
            double distance_sq = dx*dx + dy*dy;
            
            // 충돌 여부 판단: 최단 거리 <= 객체 반경 (R_obj)
            if (distance_sq <= R_obj * R_obj) {
                printf("[CRITICAL] !!! 미래 충돌 예측: T+%.1fs에 객체 %u와 충돌 (객체 Pos: %.2f, %.2f)!!!\n",
                       t_sec, o->label, obj_x, obj_y);
                return 1; // 충돌 위험 감지
            }
        }
    }
    return 0; // 충돌 위험 없음
}

    /**
    * @brief 현재 차량 상태를 기반으로 미래 경로(x, y)를 계산 (입력: -30~30 바퀴 각도)
    * @param current_speed_kph 현재 속도 (km/h)
    * @param current_steer_deg 현재 바퀴 조향각 (-30.0 ~ 30.0)
    * @param out_x 결과가 저장될 X좌표 배열
    * @param out_y 결과가 저장될 Y좌표 배열
    */
    void calc_future_path(int current_speed_kph, float current_steer_deg,
                        double out_x[POS_COUNT], double out_y[POS_COUNT]) {
        
        // 1. 배열 초기화
        memset(out_x, 0, sizeof(double) * POS_COUNT);
        memset(out_y, 0, sizeof(double) * POS_COUNT);

        // 2. 물리량 변환
        double v_mps = (double)current_speed_kph * KPH_TO_MPS;
        double steer_angle = (double)current_steer_deg;

        // 각도가 너무 작으면(직진) 0으로 처리 (노이즈 제거)
        if (fabs(steer_angle) < 0.5) {
            steer_angle = 0.0;
        }

        // 3. 직진 or 곡선 판단 및 계산
        if (steer_angle == 0.0) {
            // [직진일 때]
            for (int i = 0; i < POS_COUNT; i++) {
                double t = (i + 1) * PREDICTION_DT;
                out_x[i] = v_mps * t; 
                out_y[i] = 0.0;
            }
        } 
        else {
            // [곡선일 때: 자전거 모델]
            
            // 3-1. 바퀴 각도(deg) -> 라디안(rad) 변환
            // (입력값이 이미 바퀴 각도이므로 기어비 나눗셈 불필요)
            // [주의] Carla 좌표계: -30이 왼쪽인지 오른쪽인지 확인 필요.
            // 보통 왼쪽이 +각도(반시계)여야 수학 공식과 맞습니다.
            // 만약 경로가 반대로 휘면 아래에 -를 붙이세요: -steer_angle
            double wheel_angle_rad = (steer_angle) * (M_PI / 180.0);

            // 3-2. 회전 반경 R 계산
            // tan(90도)는 무한대이므로 방어 코드 필요하지만 30도라 안전함
            double R = VEHICLE_WHEELBASE / tan(wheel_angle_rad);

            // 3-3. 각속도 omega 계산
            // 속도가 0이면 R 계산과 무관하게 제자리
            if (fabs(v_mps) < 0.1) {
                // 정지 상태면 경로 없음(0,0)
                return; 
            }
            
            double omega = v_mps / R;

            for (int i = 0; i < POS_COUNT; i++) {
                double t = (i + 1) * PREDICTION_DT;
                
                // t초 동안 회전한 총 각도
                double theta = omega * t;

                // 원형 궤적 공식 (시작점 0,0, 초기방향 x축)
                out_x[i] = R * sin(theta);
                out_y[i] = R * (1.0 - cos(theta));
            }
        }
    }
        
    // }

    // ============================================================================
    // 파이썬 자식 프로세스를 시작하는 함수
    // - 파이프 2개 생성 → fork() → 자식에서 dup2로 stdin/stdout 재지정 → execvp로 vision_server.py 실행
    // - 부모에서는 fdopen/논블로킹 설정 등 스트림 초기화
    // ============================================================================
    static int start_python_process(void) {
        // 1) 양방향 통신을 위한 파이프 2개 생성
        if (pipe(c_to_python_pipe) == -1 || pipe(python_to_c_pipe) == -1) {
            perror("pipe() failed");
            return -1;
        }

        // 2) 자식 프로세스 생성
        pid_t pid = fork();
        if (pid < 0) {               // fork 실패
            perror("fork() failed");
            return -1;
        }

        if (pid == 0) {
            // -------------------- [자식 프로세스 영역] --------------------
            // 3) 표준 입출력 재지정: 자식의 stdin ← C→Py 파이프의 읽기쪽, stdout ← Py→C 파이프의 쓰기쪽
            dup2(c_to_python_pipe[0], STDIN_FILENO);
            dup2(python_to_c_pipe[1], STDOUT_FILENO);

            // 4) 더 이상 직접 쓰지 않을 원본 fd들은 정리(자원 누수 방지)
            close(c_to_python_pipe[0]); 
            close(c_to_python_pipe[1]); 
            close(python_to_c_pipe[0]); 
            close(python_to_c_pipe[1]);

            // 5) C 실행파일 기준으로 vision_server.py의 절대경로 계산 (원래 코드 로직 재사용)
            char exe_path[1024];
            ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path)-1);
            if (len != -1) {
                exe_path[len] = '\0';
                char *bin_dir = strrchr(exe_path, '/'); if (bin_dir) *bin_dir = '\0';  // /bin 잘라내기
                char *base_dir = strrchr(exe_path, '/'); if (base_dir) *base_dir = '\0';// /base 잘라내기
                char script_path[1024];
                snprintf(script_path, sizeof(script_path), "%s/ai/vision_server.py", exe_path);

                // 6) 파이썬 스크립트 실행 (실패 시 아래로 떨어져 종료)
                char *args[] = { "python3", script_path, NULL };
                execvp(args[0], args);
            }

            // 7) 여기 도달하면 exec 실패 → 에러 로그 후 비정상 종료
            fprintf(stderr, "EXECVP or Path Calculation FAILED: %s\n", strerror(errno));
            _exit(127); // ← 자식은 반드시 _exit 사용(버퍼 중복 flush 방지)
        }

        // -------------------- [부모 프로세스 영역] --------------------
        g_py_pid = pid; // 방금 띄운 자식 PID 기록

        // 8) 부모는 자신이 쓰지 않는 파이프 방향을 정리하고, stdio 스트림으로 감싸기
        close(c_to_python_pipe[0]);                 // 부모는 C→Py 파이프의 읽기쪽 불필요
        close(python_to_c_pipe[1]);                 // 부모는 Py→C 파이프의 쓰기쪽 불필요
        pipe_from_python_fd = python_to_c_pipe[0];  // select 감시용 fd
        stream_to_python    = fdopen(c_to_python_pipe[1], "w"); // 라인 버퍼링 출력
        stream_from_python  = fdopen(pipe_from_python_fd, "r"); // 입력 스트림

        if (!stream_to_python || !stream_from_python) {
            perror("fdopen failed");
            return -1;
        }

        // 9) '\n'마다 즉시 flush 되도록 라인 버퍼링 설정
        setvbuf(stream_to_python, NULL, _IOLBF, 0);

        // 10) 파이썬→C 파이프는 논블로킹으로: select와 궁합 좋게
        fcntl(pipe_from_python_fd, F_SETFL, O_NONBLOCK);

        // 11) 자식이 죽은 상태에서 write 시 SIGPIPE로 프로세스 전체가 죽지 않도록 무시
        signal(SIGPIPE, SIG_IGN);

        printf("[C] Python child started. PID=%d\n", (int)g_py_pid);
        return 0;
    }

    // ============================================================================
    // 파이썬 자식 프로세스를 종료/정리하는 함수
    // - 스트림/파이프 정리 → SIGTERM 전송 → 짧게 대기 후 waitpid로 수거
    // ============================================================================
    static void stop_python_process(void) {
        // 1) stdio 스트림부터 닫아 내부 버퍼를 안전히 비움
        // if (stream_to_python)   { fclose(stream_to_python);   stream_to_python = NULL; }
        // if (stream_from_python) { fclose(stream_from_python); stream_from_python = NULL; }
        if (stream_to_python)   { fclose(stream_to_python);   stream_to_python = NULL; c_to_python_pipe[1] = -1; }
        if (stream_from_python) { fclose(stream_from_python); stream_from_python = NULL; python_to_c_pipe[0] = -1; }

        // 2) 로우 fd도 닫아줌 (중복 닫힘 방지 위해 -1 체크)
        if (c_to_python_pipe[0]   != -1) { close(c_to_python_pipe[0]);   c_to_python_pipe[0]   = -1; }
        if (c_to_python_pipe[1]   != -1) { close(c_to_python_pipe[1]);   c_to_python_pipe[1]   = -1; }
        if (python_to_c_pipe[0]   != -1) { close(python_to_c_pipe[0]);   python_to_c_pipe[0]   = -1; }
        if (python_to_c_pipe[1]   != -1) { close(python_to_c_pipe[1]);   python_to_c_pipe[1]   = -1; }
        pipe_from_python_fd = -1;

        // 3) 자식 프로세스가 살아 있다면 종료 신호 전달 후 수거
        if (g_py_pid > 0) {
            kill(g_py_pid, SIGTERM); // 우아한 종료 요청
            int status = 0;
            // 짧게 폴링하며 종료 기다림
            for (int i = 0; i < 10; ++i) {
                pid_t r = waitpid(g_py_pid, &status, WNOHANG);
                if (r == g_py_pid) break; // 종료됨
                usleep(100 * 1000);       // 100ms 대기
            }
            // 그래도 안 끝났으면 블록킹 wait (혹은 SIGKILL 고려 가능)
            waitpid(g_py_pid, &status, 0);
            g_py_pid = -1;
        }
    }

    /* =======================================================================================
    * @brief JSON 문자열을 파싱하여 DetectedObject 구조체 배열로 동적 할당.
    * @param json_string Python으로부터 받은 JSON 문자열.
    * @param count 파싱된 객체의 개수를 저장할 포인터.
    * @return 동적으로 할당된 DetectedObject 배열의 포인터. 사용 후 반드시 free() 해야 함.
    * 파싱 실패 시 NULL을 반환.
    * =======================================================================================*/

    DetectedObject* parse_ai_results(const char* json_string, int*count){

        if(!count) return NULL;

        //count 포인터가 가르키는 값을 0으로 초기화함, 실패 시에도 안정성 확보
        *count = 0;

        if(!json_string) return NULL;
        
        //입력받은 문자열(json_string)을 cJSON 라이브러리를 사용해 파싱함
        //결과로 JSON 구조 전체를 나타내는 cJSON 객체(트리구조)의 최상위 노드(root)를 얻음
        cJSON *root = cJSON_Parse(json_string);

        //파싱에 실패하면 NULL값 반환
        if(NULL == root){
            fprintf(stderr, "[C] Python JSON pare error\n");
            return NULL;
        }

        //root 객체에서 objects라는 key를 가진 항목을 찾음
        cJSON *objects_array = cJSON_GetObjectItemCaseSensitive(root, "objects");

        //object 항목이 JSON배열 타입이 맞는지 확인
        if(!cJSON_IsArray(objects_array)){
            fprintf(stderr, "[C] 'objects' key is not an array\n");
            cJSON_Delete(root);
            return NULL;
        }

        //배열에 몇 개의 객체가 들어있는지 확인
        int object_count = cJSON_GetArraySize(objects_array);

        //탐지된 객체가 하나도 없는 경우 NULL을 반환
        if(object_count <= 0){
            cJSON_Delete(root);
            return NULL;
        }

        //메모리 동적 할당, 탐지된 객체의 수만큼 DetectedObject 구조체 배열을 위한 메모리를 힙에 할당
        DetectedObject* result_array = (DetectedObject*)malloc(object_count * sizeof(DetectedObject));
        
        //메모리 오류 검사, 메모리가 부족하면 NULL을 반환
        if(NULL == result_array){
            fprintf(stderr, "[C] Failed to allocate memory for objects array\n");
            cJSON_Delete(root);
            return NULL;
        }

        //배열 초기화
        memset(result_array, 0, sizeof(DetectedObject) * object_count);

        //cJSON_ArrayForEach 매크로를 사용하여 배열의 모든 요소를 순회
        int i = 0;
        cJSON *element;
        cJSON_ArrayForEach(element, objects_array){

            if(!cJSON_IsObject(element))
                continue; //객체가 아니면 스킵

            cJSON *jlabel = cJSON_GetObjectItemCaseSensitive(element, "label");
            cJSON *jx     = cJSON_GetObjectItemCaseSensitive(element, "x");
            cJSON *jy     = cJSON_GetObjectItemCaseSensitive(element, "y");
            cJSON *jax    = cJSON_GetObjectItemCaseSensitive(element, "ax");
            cJSON *jay    = cJSON_GetObjectItemCaseSensitive(element, "ay");

            result_array[i].label = (unsigned char)(cJSON_IsNumber(jlabel) ? jlabel->valueint   : 0);
            result_array[i].x     = (float)(cJSON_IsNumber(jx)     ? jx->valuedouble            : 0.0f);
            result_array[i].y     = (float)(cJSON_IsNumber(jy)     ? jy->valuedouble            : 0.0f);
            result_array[i].ax    = (float)(cJSON_IsNumber(jax)    ? jax->valuedouble           : 0.0f);
            result_array[i].ay    = (float)(cJSON_IsNumber(jay)    ? jay->valuedouble           : 0.0f);
            i++;
        }

        cJSON_Delete(root);

        if(i == 0){
            free(result_array);
            return NULL;
        }

        *count = i;
        return result_array;

    }

    /* =======================================================================================
    * ===== [ADD] 헬퍼: 파이썬 analyze 요청 라인 프로토콜 전송 (GPS/STEER 포함) ==================
    *  - 목적: 필수 데이터(GPS, 스티어링)가 준비된 시점에 단 한 줄로 명령을 보냄.
    *  - 형식: C -> Py 로 "analyze {json}\n"
    *  - 주의: fflush( ) 필수 (라인버퍼링 보장)
    * ======================================================================================= */
    static int send_ai_request(FILE* to_py, const VehicleData* v) {
        if (!to_py || !v) return -1;

        /* JSON에 부동소수점 수치를 넣음 */
        int n = fprintf(to_py,
                        "analyze {\"gps\":[%.6f,%.6f],\"steer\":%.2f}\n",
                        v->gps_x, v->gps_y, v->degree);
        if (n <= 0) return -1;

        /* 매우 중요: stdio 버퍼가 파이프로 실제 전달되도록 즉시 비움 */
        fflush(to_py);
        return 0;
    }

    // static int send_save_request(FILE* to_py, const VehicleData* v, const unsigned char value,
    //                              const double* path_x, const double* path_y, int count) {
    //     if (!to_py || !v) return -1;

    //     int n = fprintf(to_py,
    //         "draw {"
    //             "\"value\":%u,"
    //             // "\"gps\":[%.6f,%.6f],"
    //             "\"speed\":%d,"
    //             "\"rpm\":%d,"
    //             "\"brake_state\":%u,"
    //             "\"gear_ratio\":%.4f,"
    //             "\"gear_state\":%d,"      /* char -> int 코드로 전송 */
    //             //"\"degree\":%.2f,"
    //             "\"throttle\":%u,"
    //             "\"tires\":[%u,%u,%u,%u]"
    //         "}\n",
    //         (unsigned)value,
    //         // v->gps_x, v->gps_y,
    //         v->speed,
    //         v->rpm,
    //         (unsigned)v->brake_state,
    //         v->gear_ratio,
    //         (int)v->gear_state,
    //         //v->degree,
    //         (unsigned)v->throttle,
    //         v->tire_pressure[0], v->tire_pressure[1],
    //         v->tire_pressure[2], v->tire_pressure[3]
    //     );
    //     if (n <= 0) return -1;

    //     fflush(to_py);
    //     return 0;
    // }

    static int send_save_request(FILE* to_py, const VehicleData* v, const unsigned char value, 
                                const double* path_x, const double* path_y, int count) {
        if (!to_py || !v) return -1;

        // 1. JSON의 앞부분 (기본 데이터) 출력 (줄바꿈 없이 "draw {" 로 시작)
        fprintf(to_py, "draw {");
        
        fprintf(to_py, "\"value\":%u,", (unsigned)value);
        fprintf(to_py, "\"speed\":%d,", v->speed);
        fprintf(to_py, "\"rpm\":%d,", v->rpm);
        fprintf(to_py, "\"brake_state\":%u,", (unsigned)v->brake_state);
        fprintf(to_py, "\"gear_ratio\":%.4f,", v->gear_ratio);
        fprintf(to_py, "\"gear_state\":%d,", (int)v->gear_state);
        fprintf(to_py, "\"throttle\":%u,", (unsigned)v->throttle);
        fprintf(to_py, "\"tires\":[%u,%u,%u,%u],", 
                v->tire_pressure[0], v->tire_pressure[1], 
                v->tire_pressure[2], v->tire_pressure[3]);

        // 2. Path X 배열 추가
        fprintf(to_py, "\"path_x\":[");
        if (path_x) {
            for (int i = 0; i < count; i++) {
                // 마지막 원소가 아니면 콤마(,) 추가
                fprintf(to_py, "%.2f%s", path_x[i], (i < count - 1) ? "," : "");
            }
        }
        fprintf(to_py, "],");

        // 3. Path Y 배열 추가
        fprintf(to_py, "\"path_y\":[");
        if (path_y) {
            for (int i = 0; i < count; i++) {
                fprintf(to_py, "%.2f%s", path_y[i], (i < count - 1) ? "," : "");
            }
        }
        fprintf(to_py, "]");

        // 4. JSON 닫기 및 줄바꿈
        fprintf(to_py, "}\n");

        // 5. 전송
        fflush(to_py);
        return 0;
    }

    /* =======================================================================================
    * ===== [ADD] 헬퍼: 파이썬 한 줄(JSON) 처리 ==============================================
    *  - 목적: Py -> C로 들어온 한 줄(JSON 문자열)을 파싱해 ai_result에 저장하고 상태 플래그 설정
    *  - 실패해도 치명적이지 않으므로 파싱 실패는 로깅 후 무시(프로토타입 전략)
    * ======================================================================================= */
    static int handle_python_line(const char* line, unsigned char* state_flag) {
        if (!line || !state_flag) return -1;

        int n = 0;
        DetectedObject *objs = parse_ai_results(line, &n);
        if(!objs || n <= 0){
            fprintf(stderr, "[C] AI parse failed or empty objects\n");
            *state_flag |= AI_RESEULT_ERROR_FLAG;   // AI 결과 에러 플래그
            return -1;
        }

        if(g_ai_objs) free(g_ai_objs);
        g_ai_objs = objs;
        g_ai_count = n;

        *state_flag |= AI_RESULT_READY_FLAG;   // AI 결과 수신 상태 완료
        return 0;
    }

    static int wait_python_done(FILE* in, int fd, int timeout_ms)
    {
        if (!in || fd < 0) return -1;

        while (1) {
            // 1) 읽기 가능해질 때까지 select()로 대기 (타임아웃 적용)
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);

            struct timeval tv, *ptv = NULL;
            if (timeout_ms > 0) {
                tv.tv_sec  = timeout_ms / 1000;
                tv.tv_usec = (timeout_ms % 1000) * 1000;
                ptv = &tv;
            }
            int r = select(fd + 1, &rfds, NULL, NULL, ptv);
            if (r < 0) {
                if (errno == EINTR) continue;     // (신호로 깨어나면 재시도)
                perror("[C] wait_python_done select");
                return -1;
            }
            if (r == 0) {
                // 2) 타임아웃: 더 기다리지 않음
                return 1;
            }

            // 3) 읽을 데이터가 있음 → 라인 단위로 모두 읽어 "done" 확인
            if (FD_ISSET(fd, &rfds)) {
                char line[4096];
                while (fgets(line, sizeof(line), in)) {
                    // (핵심) 정확히 "done"으로 시작하는 라인 수신 시 완료
                    if (strncmp(line, "done", 4) == 0) return 0;
                    // 그 외 라인은 무시(필요하면 여기서 별도 파서 호출 가능)
                }
                // 4) 더 이상 읽을 게 없고 EOF면 자식 종료로 판단
                if (feof(in)) return -2;

                // 논블로킹/부분읽기 등으로 인한 임시 에러 플래그 정리 후 루프 계속
                clearerr(in);
            }
        }
    }

    // --- 3. main 함수: 모든 코드의 시작점 ---
    int main() {
        // --- 2-1. 파이프(Pipe) 생성 ---
        if (start_python_process() < 0) {
            fprintf(stderr, "[C] FATAL: failed to start python child\n");
            return EXIT_FAILURE;
        }

        //CAN 버스 초기화
        int can_fd = can_init("can0");
        if(can_fd < 0){
            fprintf(stderr, "[C] FATAL: Failed to initialize CAN bus. Exiting.\n");
            exit(EXIT_FAILURE);
        }

        // --- 4-3. 상태 관리를 위한 변수 선언 ---
        VehicleData vehicle_data = {0}; // 차량 데이터를 저장할 구조체
        CANMessage can_message = {0};   // CAN통신 데이터 프레임
        unsigned char state_flag = 0;   // 상태 플래그
        unsigned char ai_state_flag = 0;// AI 분석 결과 플래그
        unsigned char state_flag2 = 0;

        unsigned char car_state_flag = 0;  // 자동차 상태 확인 플래그

        //파이썬에게 보낼 좌표 배열
        double PosX_array[POS_COUNT];
        double PosY_array[POS_COUNT];

        struct timespec request_time, complete_time;
        long diff_ns = 0;

        printf("[C] Main process start. Child PID: %d\n", (int)g_py_pid);

        sleep(2); //시작 대기 시간

        // --- 4-4. 메인 이벤트 루프: 장치의 심장 박동 ---
        while (1) {
                // --- A. 필수 데이터 수집 및 파이썬 요청 단계 ---
            // ai분석 요청을 하지 않았다면
            if((ai_state_flag & AI_REQUEST_FLAG) != AI_REQUEST_FLAG){
                //CAN 통신으로 필수 데이터를 받지 않았다면
                if((state_flag & AI_AVAILABLE) != AI_AVAILABLE){
                
                    //GPS 데이터를 받지 않았다면
                    if((state_flag & GPS_AVAILABLE) != GPS_AVAILABLE){
                        //X좌표 데이터를 받지 않았다면
                        if((state_flag & GPS_XDATA_FLAG) != GPS_XDATA_FLAG){
                            //X좌표 데이터 요청
                            if(can_request_pid(PID_GPS_XDATA) < 0){
                                perror("[C] PID_GPS_XDATA request error");
                            }
                            else{
                                printf("[C] GPS_XDATA_REQUEST\n");
                            }
                        }
                        //x좌표 데이터를 받았다면
                        else{
                            //Y좌표 데이터 요청
                            if((state_flag & GPS_YDATA_FLAG) != GPS_YDATA_FLAG){   // ✅ FLAG로 검사
                                if(can_request_pid(PID_GPS_YDATA) < 0){
                                    perror("[C] PID_GPS_YDATA request error");
                                }
                                else{
                                    printf("[C] GPS_YDATA_REQUEST\n");
                                }
                            }
                        }
                    }
                    //GPS 데이터를 받았다면
                    else{
                        //스티어링 데이터 요청
                        if(can_request_pid(PID_STEERING_DATA) < 0){
                            perror("[C] STEERING_DATA request error");
                        }
                        else{
                            printf("[C] STEERING_DATA_REQUEST\n");
                        }
                    }
                }

                //필수 데이터를 모두 수집했으면
                else{

                    //여기에 파이썬 실행 코드 추가, GPS좌표와 스티어링 데이터를 넘김
                    // ===== [ADD] 필수 두 데이터(GPS, 조향각)가 준비되면, 파이썬에 분석 명령 전송 =====
                    if (send_ai_request(stream_to_python, &vehicle_data) == 0) {
                        ai_state_flag |= AI_REQUEST_FLAG;   // 중복 요청 방지
                    } else {
                        perror("[C] send_ai_request failed");
                    }
                }
            }

            //ai 분석 요청을 했다면
            else{
                //나머지 CAN 통신 요청
                // (요구사항: AI가 돌고 있는 동안에도 CAN 수집은 계속된다)
                // 필요시 여기에서 엔진, 속도, 브레이크, 타이어 등 추가 PID를 라운드-로빈으로 요청해도 됨.
                // 예시(프로토타입): 속도/엔진RPM 라운드로 요청


                // ======= [ADD] AI가 처리할동안 작업 수행 부분 ================================
                
                //차의 예상 경로 체크
                calc_future_path(vehicle_data.speed, vehicle_data.degree, PosX_array, PosY_array);

                //다른 CAN 데이터를 받지 못핬다면
                if((state_flag & DATA_AVAILABLE) != DATA_AVAILABLE){
                    
                    //리스트를 순회하며 아직 받지 못한 PID를 찾음
                    for(unsigned char i = 0; i < num_pids_to_request; i++){
                        //이번 순서에 확인할 요청 정보 가져오기
                        const CANRequest* current_req = &pids_to_request[next_pid_index];
                        
                        //이 요청에 해당하는 데이터가 아직 수신되지 않았으면
                        if((state_flag & current_req->flag) != current_req->flag){
                            
                            //CAN버스로 데이터 요청
                            if(can_request_pid(current_req->pid) < 0){
                                perror("[C] CAN_Data_request failed");
                            }
                            
                            //인덱스 업데이트
                            next_pid_index = (next_pid_index + 1) % num_pids_to_request;
                            break;
                        }
                        //이미 플래그가 세워져 있으면 인덱스 1 증가시킴
                        next_pid_index = (next_pid_index + 1) % num_pids_to_request;
                        
                    }
                }

                //쓰로틀 업데이트(임시)
                else if((state_flag2 & THROTTLE_DATA_FLAG) == 0x00){
                    if(can_request_pid(PID_THROTTLE_DATA) < 0){
                        perror("[C] CAN_Data_request failed");
                    }
                }

                // ===========================================================================
                
                
                
            }

            // --- B. I/O 멀티플렉싱(select) 단계 ---
            /* ===== [ADD] select()로 파이썬 파이프 + CAN 소켓을 동시에 감시 ==================
                *  - 타임아웃: 50ms (프로토타입 기준 반응성/부하 균형)
                *  - 준비된 FD만 비동기적으로 처리 → 한 루프에서 가능한 한 많이 소화
                * ========================================================================= */
            fd_set rfds;
            FD_ZERO(&rfds);
            int maxfd = -1;
            if (pipe_from_python_fd >= 0) {
                FD_SET(pipe_from_python_fd, &rfds);
                if (pipe_from_python_fd > maxfd) maxfd = pipe_from_python_fd;
            }
            if (can_fd >= 0) {
                FD_SET(can_fd, &rfds);
                if (can_fd > maxfd) maxfd = can_fd;
            }
            if (maxfd < 0) {
                // 감시할 FD가 없다면 잠깐 쉰 뒤 다음 루프로
                usleep(50 * 1000);
                continue;
            }

            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 50 * 1000; // 50 ms

            int ready = select(maxfd + 1, &rfds, NULL, NULL, &tv);
            if (ready < 0) {
                if (errno == EINTR) continue; // 신호로 깨어남(무시)
                perror("[C] select");
                break;
            }
            if (ready == 0) {
                // 타임아웃: 주기 작업 자리(하트비트 등)
                printf("timeout\n");
                // continue;
            }

            // >>> 1) 파이썬 결과 수신 (라인 단위 JSON)
            if (pipe_from_python_fd >= 0 && FD_ISSET(pipe_from_python_fd, &rfds)) {
                /* 주의: fd는 논블로킹. stream_from_python은 stdio 버퍼를 쓰므로
                    fgets가 즉시 NULL을 줄 수 있음(EAGAIN). 이는 '아직 한 줄이 안 채워짐' 의미 */
                char line[4096];
                while (fgets(line, sizeof(line), stream_from_python)) {
                    /* vision_server.py는 결과를 한 줄 JSON으로 print하고 flush함 */
                    handle_python_line(line, &ai_state_flag);
                    /* 파이썬이 여러 줄을 연속적으로 보낼 수 있으므로 while로 드레인 */
                }

                /* EOF(파이썬 종료) 감지 */
                if (feof(stream_from_python)) {
                    fprintf(stderr, "[C] Python EOF detected. Restarting child...\n");

                    // 1) AI 결과 동적 메모리/상태 정리 (누수/유효하지 않은 포인터 참조 방지)
                    if (g_ai_objs) { free(g_ai_objs); g_ai_objs = NULL; g_ai_count = 0; }
                    ai_state_flag = 0; // AI 결과 준비 플래그 초기화

                    // 2) 현 자식 프로세스 및 I/O 정리
                    stop_python_process();

                    // 3) 짧은 백오프(옵션): 연속 크래시 시 과도한 재시작을 피함
                    sleep(1);

                    // 4) 재시작 시도
                    if (start_python_process() < 0) {
                        fprintf(stderr, "[C] Restart failed. Will retry in 3s.\n");
                        sleep(3);
                        // 재시작 실패 시 다음 루프에서 다시 시도하도록 continue
                        continue;
                    }

                    // 5) 스트림 에러 상태 초기화 (안전조치)
                    clearerr(stream_from_python);
                    continue; // 재시작 직후 루프 재개
                }
                clearerr(stream_from_python); // EAGAIN 등 클리어
            }

            // >>> 2) CAN 프레임 수신 (있을 때 모두 드레인)
            if (can_fd >= 0 && FD_ISSET(can_fd, &rfds)) {
                while (1) {
                    int r = can_receive_message(&can_message);
                    if (r < 0) {
                        // 심각한 소켓 에러 가능 (프로토타입: 경고만)
                        perror("[C] can_receive_message");
                        break;
                    } else if (r == 0) {
                        // 읽을 데이터 없음(논블로킹): 루프 종료
                        break;
                    } else {
                        // 정상 프레임 1개 수신 → 파싱 및 상태 플래그 갱신
                        can_parse_and_update_data(&can_message, &vehicle_data, &state_flag, &state_flag2);
                    }
                }
                printf("[DEBUG] Flags: state_flag=0x%02X, state_flag2=0x%02X, ai_state_flag=0x%02X\n",
                state_flag, state_flag2, ai_state_flag);
            }

            // >>> 3) 완료 조건 체크: AI 결과 + CAN 측 “완료 세트” 충족 시 제어 로직 실행
            /* COMPLETE_DATA_FLAG는 hardware.h에 정의된 전체 데이터 집합 플래그임.
                (ENGINE_SPEED, VEHICLE_SPEED, GEAR_STATE, GPS, STEERING, BRAKE, TIRE 등)
                프로토타입에서는 이 완전 세트를 만족했을 때 한 번 제어 로직을 실행하도록 구성. */
            if ( ((ai_state_flag & AI_RESULT_READY_FLAG) == AI_RESULT_READY_FLAG) &&
                    ((state_flag & COMPLETE_DATA_FLAG) == COMPLETE_DATA_FLAG) &&
                    ((state_flag2 & 0x01) == 0x01) ) {

                
                //확인용 로그 출력
                printf("[Path Prediction] ----------------------\n");
                for(int i=0; i<POS_COUNT; i++){
                    double t = (i+1) * PREDICTION_DT;
                    printf("T+%.1fs: (%.2f, %.2f)\n", t, PosX_array[i], PosY_array[i]);
                }
                printf("----------------------------------------\n");

            // ======= [ADD] 최종 제어 로직 지점 ==========================================
                if (g_ai_objs && g_ai_count > 0) {
                    for (int i = 0; i < g_ai_count; ++i) {
                        const DetectedObject *o = &g_ai_objs[i];
                        // TODO: 제어 로직 
                        printf("[AI] L=%u x=%.2f y=%.2f ax=%.2f ay=%.2f\n",
                                    o->label, o->x, o->y, o->ax, o->ay);
                        //AI로부터 받은 JSON파일 분석
                        // TODO: 동적 배열 for문으로 순회하면서 거리 및 좌표를 통한 거리 계산
                        /*
                        받는 데이터 형식
                        typedef struct{
                            unsigned char label;
                            float x;
                            float y;
                            float ax;
                            float ay;
                        }DetectedObject;
                        */
                        //ai의 label 구성
                        /*
                        0 = car
                        1 = truck
                        2 = construction_vehicle
                        3 = bus
                        6 = motorcycle
                        7 = bicycle
                        8 = pedestrian // 보행자
                        */
                        float distance = hypotf(o->x, o->y);
                        
                        //거리가 임계값 안이라면
                        if(distance <= MAX_DISTANCE){
                            switch(o->label){
                                case LABEL_CAR:
                                    break;

                                case LABEL_TRUCK:
                                    car_state_flag |= DETECT_TRUCK;
                                    break;

                                case LABEL_CONSTRUCTION_TRUCK:
                                    car_state_flag |= DETECT_TRUCK;
                                    break;

                                case LABEL_BUS:
                                    break;

                                case LABEL_TRAILER:
                                    break;

                                case LABEL_BARRIER:
                                    break;

                                case LABEL_MOTOCYCLE:
                                    car_state_flag |= DETECT_ODOBANGS;
                                    break;

                                case LABEL_BICYCLE:
                                    car_state_flag |= DETECT_ODOBANGS;
                                    break;

                                case LABEL_PEDESTRIAN:
                                    car_state_flag |= DETECT_HUMAN;
                                    break;

                                case LABEL_TRAFFIC_CONE:
                                    break;

                                default:
                                    break;
                            }
                        }

                        
                    }
                    
                    // 2. 미래 충돌 예측 검사 및 플래그 갱신
                    if (check_collision_risk(g_ai_objs, g_ai_count, PosX_array, PosY_array, POS_COUNT)) {
                        car_state_flag |= DETECT_CRASH_RISK;
                    }
                                    

                }
                //=========가속도 저장 부분=============
                double t_now = now_sec(); //현재 시간 측정
                double v_now_kph = (double)vehicle_data.speed; //속도 int -> double 형변환
                spmon_push(&g_spmon, v_now_kph, t_now); //속도, 시간 추가

                //직전 샘플과 비교하여 가속도 계산 
                double v_prev_kph, t_prev;
                if(spmon_get_past(&g_spmon, 1, &v_prev_kph, &t_prev) == 0){
                    double dv_mps = (v_now_kph - v_prev_kph) * KPH_TO_MPS;
                    double dt     = t_now - t_prev;
                    if(dt > 0.0){
                        double a_mps2 = dv_mps / dt;//가속도 계산
                        
                        //가속도 감지 후 동작

                        //급가속
                        if (a_mps2 >= ACCEL_THRESH_MPS2) {
                            printf("[EVENT] 급가속 감지: a=%.2f m/s^2 (%.1f→%.1f km/h, dt=%.2fs)\n",
                                a_mps2, v_prev_kph, v_now_kph, dt);
                            // TODO: 급가속 플래그 On
                            car_state_flag |= ACCELRATION;
                            

                        //급감속
                        } else if (a_mps2 <= DECEL_THRESH_MPS2) {
                            printf("[EVENT] 급감속 감지: a=%.2f m/s^2 (%.1f→%.1f km/h, dt=%.2fs)\n",
                                a_mps2, v_prev_kph, v_now_kph, dt);
                            // TODO: 급감속 플래그 on
                            car_state_flag |= DECELERATION;
                        }

                    }
                }

                //타이어 펑크 검출
                for(int i = 0; i < 4; i++){
                    if(vehicle_data.tire_pressure[i] < TIRE_PRESSURE_THRESHOLD){
                        car_state_flag |= DETECT_FUNK;
                    }
                }
                
                if (send_save_request(stream_to_python, &vehicle_data, car_state_flag, PosX_array, PosY_array, POS_COUNT) == 0) {
                    // 최대 3000ms(3초) 동안 "done" 대기. 0을 주면 무제한 대기.
                    int wr = wait_python_done(stream_from_python, pipe_from_python_fd, 0);
                    if (wr == 0) {
                        printf("[C] Python done OK\n");
                    } else if (wr == 1) {
                        fprintf(stderr, "[C] Python done timeout\n");
                    } else if (wr == -2) {
                        fprintf(stderr, "[C] Python EOF (child exited)\n");
                    } else {
                        fprintf(stderr, "[C] Python done wait error\n");
                    }

                } else {
                        perror("[C] send_save_request failed");
                }

                //메모리 해제
                if (g_ai_objs) { free(g_ai_objs); g_ai_objs = NULL; g_ai_count = 0; }
                state_flag = 0;
                state_flag2 = 0;
                ai_state_flag = 0;
                car_state_flag = 0;
                printf("\nfinish one cycle, next cycle will be started.\n");

            }

            if((ai_state_flag & AI_RESEULT_ERROR_FLAG) == AI_RESEULT_ERROR_FLAG){
                //메모리 해제
                if (g_ai_objs) { free(g_ai_objs); g_ai_objs = NULL; g_ai_count = 0; }
                
                printf("\nAI error occurred, next cycle will be started.\n");

                car_state_flag |= 0x80; //AI 에러 플래그

                if (send_save_request(stream_to_python, &vehicle_data, car_state_flag, PosX_array, PosY_array, POS_COUNT) == 0) {
                    // 최대 3000ms(3초) 동안 "done" 대기. 0을 주면 무제한 대기.
                    int wr = wait_python_done(stream_from_python, pipe_from_python_fd, 0);
                    if (wr == 0) {
                        printf("[C] Python done OK\n");
                    } else if (wr == 1) {
                        fprintf(stderr, "[C] Python done timeout\n");
                    } else if (wr == -2) {
                        fprintf(stderr, "[C] Python EOF (child exited)\n");
                    } else {
                        fprintf(stderr, "[C] Python done wait error\n");
                    }

                } else {
                        perror("[C] send_save_request failed");
                }
                state_flag = 0;
                state_flag2 = 0;
                ai_state_flag = 0;
                car_state_flag = 0;

            }

            // ===========================================================================
            // fprintf(stdout,
            //         "[C] CONTROL: AI+CAN 완료. speed=%d rpm=%d gear=%c gps(%.6f,%.6f) steer=%.3f\n",
            //         vehicle_data.speed, vehicle_data.rpm, vehicle_data.gear_state,
            //         vehicle_data.gps_x, vehicle_data.gps_y, vehicle_data.degree);
            // fflush(stdout);

            // 다음 사이클 준비: AI 관련 플래그/결과만 리셋 → CAN은 계속 최신 값 유지
            
            

            // 필요하면 필수 두 데이터(GPS/STEER) 플래그만 리셋해서,
            // 다시 두 데이터를 확보한 뒤 새로운 analyze 라운드를 도는 정책도 가능.
            

        } // --- while(1) 루프 끝 ---

        printf("\n[C] Main process finished. Cleaning up resources.\n");
        stop_python_process();              // 파이썬 자식/파이프/스트림 한 번에 정리
        return 0;
    }