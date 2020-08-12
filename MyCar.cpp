#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include "RuleBasedDriving.h"

using namespace std;
using namespace Car;

bool is_debug = false;
int recovery_count = 0;
int accident_count = 0;
bool collisionFlag = false;

ControlValues control_driving(CarStateValues sensing_info)
{
	if (is_debug)
	{
		cout << "=========================================================" << endl;
		cout << "[MyCar] to middle: " << sensing_info.to_middle << endl;

		cout << "[MyCar] collided: " << sensing_info.collided << endl;
		cout << "[MyCar] car speed: " << sensing_info.speed << "km/h" << endl;

		cout << "[MyCar] is moving forward: " << sensing_info.moving_forward << endl;
		cout << "[MyCar] moving angle: " << sensing_info.moving_angle << endl;
		cout << "[MyCar] lap_progress: " << sensing_info.lap_progress << endl;

		cout << "[MyCar] track_forward_angles: ";
		for (int i = 0; i < sensing_info.track_forward_angles.size(); i++)
		{
			cout << sensing_info.track_forward_angles[i] << ", ";
		}
		cout << endl;

		cout << "[MyCar] track_forward_obstacles: ";
		for (int i = 0; i < sensing_info.track_forward_obstacles.size(); i++)
		{
			cout << "{dist: " << sensing_info.track_forward_obstacles[i].dist 
				 << ", to_middle: " << sensing_info.track_forward_obstacles[i].to_middle << "}, ";
		}
		cout << endl;

		cout << "[MyCar] opponent_cars_info: ";
		for (int i = 0; i < sensing_info.opponent_cars_info.size(); i++)
		{
			cout << "{dist: " << sensing_info.opponent_cars_info[i].dist 
				 << ", to_middle: " << sensing_info.opponent_cars_info[i].to_middle 
				 << ", speed: " << sensing_info.opponent_cars_info[i].speed << "km/h}, ";
		}
		cout << endl;

		cout << "[MyCar] distance_to_way_points: ";
		for (int i = 0; i < sensing_info.distance_to_way_points.size(); i++)
		{
			cout << sensing_info.distance_to_way_points[i] << ", ";
		}
		cout << endl;

		cout << "=========================================================" << endl;
	}

	ControlValues car_controls;
	// ===========================================================
	// Area for writing code about driving rule ==================
	// ===========================================================
	// Editing area starts from here
	//	
    // Moving straight forward
      // sensing_info
    float set_brake = 0.0f; // 0~1로, 0이면 브레이크를 안밟은것, 1이면 밟은것
    float set_throttle = 0.9f; // -1~1 속도가 빠르면 트랙배열의 뒤에것을 참고하도록 구현되어져야 한다.
    
    /*
     * 1)
     *
     */
     // track_forward_obstacles ==> 전방 장애물 위치 (정보)
     // Target_Point == > 내가 가려고 하는 곳0.75( to_middle - avoid_width - to_middle )
     // 삼각함수얘기가
     // 나옴.. 피해야하는 각도만큼의 계산이 필요해보임.

    int angle_num = (int)(sensing_info.speed / 90); // 이 숫자가 도로정보 관련 배열들의 인덱스를 선택하게하는 값이 된다. ( 속도가 빠를수록 높은 인덱스를
    float ref_angle = sensing_info.track_forward_angles[angle_num]; // 핸들값 변경 ( 속도가 빠르면 배열 뒤쪽을 바라봄 )
    float middle_add = (sensing_info.to_middle / 70) * -1; // 양수이면 도로의 오른쪽에 있는 것이니, 핸들을 왼쪽으로 꺾어야한다. ( 중앙에 가까울 수록 0에
    float set_steering = (ref_angle - sensing_info.moving_angle) / (180 - sensing_info.speed); // 180은 최대속도인데, 우리는
    set_steering += middle_add;


    // ================================= 장애물 처리=====================================
    if (sensing_info.track_forward_obstacles.size() > 0) {
        Car::ObstaclesInfo fwd_obstacle = sensing_info.track_forward_obstacles[0]; // 여기서 0이 장애물 정보를
                                                                             // 갖고 있는 0번째
                                                                             // 배열임.
        if (fwd_obstacle.dist < 60 && fwd_obstacle.dist > 0 && abs(fwd_obstacle.to_middle) < 8.0) {
            // 도로밖은 제외하고, 전방 0~60미터까지만 장애물에 대한 처리를 하는 것 ( 도로폭은 8임)

        }

        float avoid_width = 4.7f; // 피해야하는 길이 생각해야함. ( 2개인것도있고 3개가 띄엄띄엄 있는 경우도 있음 )
        float diff = fwd_obstacle.to_middle - sensing_info.to_middle;

        if (abs(diff) < avoid_width) {
            ref_angle = (float)(abs(atan((diff - avoid_width) / fwd_obstacle.dist) * (180 / 3.14159265358979f)));
            middle_add = 0;
            if (diff > 0) {
                ref_angle *= -1; // 여기까지 코드가 구현되면 불필요한 경우에도 가게됨. 그래서 장애물과의 거리를 체크해야함. -1로 두면 왼쪽으로만 감.
                set_steering = (ref_angle - sensing_info.moving_angle) / (180 - sensing_info.speed);
            }
        }
    }
    // ===============================================================================

    // ================================= 상대차량 위치파악 및 스티어링 조작 =====================================
    if (sensing_info.opponent_cars_info.size() > 0) {
        Car::CarsInfo opp_car = sensing_info.opponent_cars_info[0];
        if (opp_car.dist < 20 && opp_car.dist > -5) { // 해당 범위 내에 있는 상대차만 고려하기. 전방

            float offset = (8 * -abs(opp_car.to_middle)); // 간격을 구하기 위한 값 ( 상대차 ) , 8은 도로폭의 절반값
            if (opp_car.to_middle > sensing_info.to_middle) { // 상대차보다 왼편에 있는 경우
                middle_add = ((sensing_info.to_middle + offset) / 70) * -1;
            }
            else {
                middle_add = ((sensing_info.to_middle - offset) / 70) * -1;
            }
            set_steering += middle_add;
        }
    }
    // ==============================================================================================

    // 각 도로의 정보 배열마다 몇미터인지에 대한 정보를 확인할 수 있음. ( 위의 코드는 각도를 얻을 수 있음. 필요하면 활용하고 필요없으면
    // 안해도됨 )

    //sensing_info.distance_to_way_points.get(0);

    // ====================== 급커브 구간에 대한 설정 =================
    bool full_throttle = true;
    bool emergency_brake = false;
    
    int road_range = (int)(sensing_info.speed / 30); // 현재 내 스피드를 30으로 나누어 줌
    for (int i = 0; i < road_range; i += 1) {
        float fwd_angle = abs(sensing_info.track_forward_angles[i]);
        if (fwd_angle > 50) { // 커브값이 50도 이상인 경우 full throttle 실시
            full_throttle = false;
        }
        if (fwd_angle > 90) {
            emergency_brake = true;
            break;
        }
    }

    if (!full_throttle) {
        if (sensing_info.speed > 130) { // 속도가 130이 넘어간 경우에는 throttle을 반으로 줄임.
            set_throttle = 0.5f;
        }
        if (sensing_info.speed > 120) { // 속도가 120이 넘어가있는 경우 브레이크
            set_brake = 1;
        }
    }

    if (emergency_brake) {
        if (set_steering > 0) {
            set_steering += 0.3f; // emergency_brake인 경우 핸들을 더 꺾음
        }
        else {
            set_steering -= 0.3f;
        }
    }

    // ==================================================================
    // 급커브 계산

    // ============================ 충돌했을 때 처리 ==============================
    if (sensing_info.lap_progress > 0.5 && !collisionFlag && (sensing_info.speed < 1.0 && sensing_info.speed > -1.0)){ // sensing_info.lap_progress는 첫 시작을 제외하기 위함                                                            
        accident_count += 1;
        cout << accident_count << endl;
    }

    if (accident_count > 6) {
        collisionFlag = true;
    }

    if (collisionFlag) {
        set_steering = 0;
        set_brake = 0;
        set_throttle = -1;
        recovery_count += 1; // 후진을 언제까지할 것인지
    }

    if (recovery_count > 20) {
        collisionFlag = false;
        recovery_count = 0;
        accident_count = 0;
        set_steering = 0;
        set_brake = 0;
        set_throttle = 0;
    }

    // Moving straight forward
    car_controls.steering = set_steering;
    car_controls.throttle = set_throttle;
    car_controls.brake = set_brake;


	if (is_debug)
	{
		cout << "[MyCar] steering:" << car_controls.steering << ", throttle:" << car_controls.throttle << ", brake:" << car_controls.brake << endl;
	}
	//
	// Editing area ends
	// ===========================================================

	return car_controls;
}

int main()
{
	// ===========================================================
	// Don't remove below area. ==================================
	// ===========================================================
	cout << "[MyCar] Start Bot!" << endl;
	StartDriving(control_driving);
	cout << "[MyCar] End Bot!" << endl;
	// ==========================================================

	return 0;
}
