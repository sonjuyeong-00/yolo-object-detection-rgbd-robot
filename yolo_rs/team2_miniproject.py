import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

import DR_init
from yolo_rs.gripper_drl_controller import GripperController

#--------------------------sys
import threading
import queue
#--------------------------sys

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
VELOCITY, ACC = 100, 100

#--------------------------sys
# move reaction
DR_MV_RA_DUPLICATE = 0

GRIPPER_GRAB_POS = 300

# RH-P12-RN 기준(대략): 스트로크 0~106mm, Goal Position 0~1150
GRIPPER_STROKE_MAX_MM = 106.0
GRIPPER_PULSE_MAX = 1150
GRIPPER_GRAB_MARGIN_MM = 2.0     # 물체 폭보다 조금 더 닫기

# 카메라 좌표 -> 로봇 좌표 보정 (필요 시 현장에 맞게 수정)
# 입력: 카메라 좌표계(mm). 출력: 로봇 좌표계(mm)
CAM_TO_ROBOT_OFFSET_X_MM = 530.0
CAM_TO_ROBOT_OFFSET_Y_MM = -170.0
CAM_TO_ROBOT_OFFSET_Z_MM = 750.0
CAM_TO_ROBOT_SIGN_X = 1.0   # robot_x =  sign_x * camera_y
CAM_TO_ROBOT_SIGN_Y = 1.0   # robot_y =  sign_y * camera_x
CAM_TO_ROBOT_SIGN_Z = -1.0  # robot_z =  sign_z * camera_z (inverse)

# 고정 Place 위치 (mm). 실제 작업 환경에 맞게 반드시 조정
PLACE_X_MM = 300.0
PLACE_Y_MM = 200.0
PLACE_Z_MM = 250.0
#--------------------------sys

def camera_point_to_robot_mm(sp: PointStamped):
    """카메라 좌표(PointStamped, m)를 로봇 좌표(mm)로 변환."""
    x_mm = sp.point.x * 1000.0
    y_mm = sp.point.y * 1000.0
    z_mm = sp.point.z * 1000.0
    final_x = CAM_TO_ROBOT_OFFSET_X_MM + (CAM_TO_ROBOT_SIGN_X * y_mm)
    final_y = CAM_TO_ROBOT_OFFSET_Y_MM + (CAM_TO_ROBOT_SIGN_Y * x_mm)
    final_z = CAM_TO_ROBOT_OFFSET_Z_MM + (CAM_TO_ROBOT_SIGN_Z * z_mm)
    if final_z <= 110.0:
        final_z = 110.0
    return final_x, final_y, final_z

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("robot_controller_node")

        self.get_logger().info("ROS 2 구독자 설정을 시작합니다.")
        
        self.gripper = None
        try:
            from DSR_ROBOT2 import wait
            self.gripper = GripperController(node=self, namespace=ROBOT_ID)
            wait(2)
            if not self.gripper.initialize():
                self.get_logger().error("그리퍼 초기화 실패. 종료합니다.")
                raise RuntimeError("그리퍼 초기화 실패.")
            self.get_logger().info("그리퍼를 활성화합니다.")
            self.gripper_is_open = True
            wait(10)
            self.get_logger().info("그리퍼를 초기화이동: 0")
            self.gripper.move(50)
            wait(2)
            
        except Exception as e:
            self.get_logger().error(f"그리퍼 세팅중에 실패하였습니다.: {e}")
            raise

        self.get_logger().info("ROS 2 구독자와 로봇 컨트롤러가 초기화되었습니다.")

        # YOLO 선택 결과 구독
        self.selected_point_sub = self.create_subscription(
            PointStamped, '/snack_boxes/selected_point', self.selected_point_callback, 10
        )
        self.selected_width_sub = self.create_subscription(
            Float32, '/snack_boxes/selected_width', self.selected_width_callback, 10
        )
        self.latest_selected_point = None
        self.latest_selected_width = None
        self.new_selection_event = threading.Event()
        self.selection_consumed = False
        self.last_selected_msg_time = None
        self.selection_reset_sec = 0.5

    def terminate_gripper(self):
        if self.gripper:
            self.gripper.terminate()

    def selected_width_callback(self, msg: Float32):
        self.latest_selected_width = float(msg.data)

    def selected_point_callback(self, msg: PointStamped):
        self.latest_selected_point = msg
        self.new_selection_event.set()
        self.last_selected_msg_time = self.get_clock().now()

    def width_m_to_pulse(self, width_m: float, extra_mm: float = 0.0) -> int:
        if width_m is None:
            return GRIPPER_GRAB_POS
        width_mm = max(0.0, width_m * 1000.0 + extra_mm)
        width_mm = min(width_mm, GRIPPER_STROKE_MAX_MM)
        pulse = int(round(width_mm * GRIPPER_PULSE_MAX / GRIPPER_STROKE_MAX_MM))
        pulse = max(0, min(pulse, GRIPPER_PULSE_MAX))
        return pulse


    def pick_move_robot_and_control_gripper(self, x, y, z, width=None, depth=None):
        from DSR_ROBOT2 import get_current_posx, movel, wait, movej
        from DR_common2 import posx, posj
        try:
            current_pos = get_current_posx()[0]
            target_pos_list_up = [x, y, z + 100, current_pos[3], current_pos[4], current_pos[5]]    #   Cuurent pos[3~5] 현재 자세값
            target_pos_list = [x, y, z, current_pos[3], current_pos[4], current_pos[5]]             #   Cuurent pos[3~5] 현재 자세값
            p_start = posj(0, 0, 90, 0, 90, 0)                                                      #   초기자세 JOINT 값
            self.get_logger().info(f"{current_pos} / 현재위치 값")
            
            # TARGET POS UP 위치 L무브
            self.get_logger().info(f"{target_pos_list_up} / 픽-업 위치 이동합니다.")
            movel(posx(target_pos_list_up), vel=VELOCITY, acc=ACC,  radius= 20.0, ra=DR_MV_RA_DUPLICATE)              
            #wait(0.5)

            # TARGET POS DOWN 위치까지 L무브
            self.get_logger().info(f"{target_pos_list} / 픽 위치로 이동합니다.")
            movel(posx(target_pos_list), vel=VELOCITY, acc=ACC,  radius= 0, ra=DR_MV_RA_DUPLICATE)
            wait(0.5)

            # GRIPPER - GRAB
            self.get_logger().info("그리퍼를 그랩합니다.")
            if width is not None:
                grab_pulse = self.width_m_to_pulse(width, extra_mm=-GRIPPER_GRAB_MARGIN_MM)
                self.get_logger().info(f"그랩 pulse: {grab_pulse}")
                self.gripper.move(grab_pulse)
            else:
                self.gripper.move(GRIPPER_GRAB_POS)
            wait(5)
            
            self.get_logger().info("오브젝트를 잡았습니다.")
             
            # TARGET POS UP 위치 L무브
            self.get_logger().info(f"{target_pos_list_up} / 픽-업 위치로 이동합니다.")
            movel(posx(target_pos_list_up), vel=VELOCITY, acc=ACC,  radius= 5.0, ra=DR_MV_RA_DUPLICATE)    
            #wait(0.5)
            

        except Exception as e:
            self.get_logger().error(f"로봇 이동 및 그리퍼 제어 중 오류 발생: {e}")
            
    def place_move_robot_and_control_gripper(self, x, y, z, width=None, depth=None):
        from DSR_ROBOT2 import get_current_posx, movel, wait, movej
        from DR_common2 import posx, posj
        try:
            current_pos = get_current_posx()[0]
            target_pos_list_up = [x, y, z + 100, current_pos[3], current_pos[4], current_pos[5]]    #   Cuurent pos[3~5] 현재 자세값
            target_pos_list = [x, y, z, current_pos[3], current_pos[4], current_pos[5]]             #   Cuurent pos[3~5] 현재 자세값
            p_start = posj(0, 0, 90, 0, 90, 0)                                                      #   초기자세 JOINT 값
            self.get_logger().info(f"{current_pos} / 현재위치 값 ")
            # TARGET POS UP 위치 L무브
            self.get_logger().info(f"{target_pos_list} / 플레이스-업 위치로 이동합니다. ")
            movel(posx(target_pos_list_up), vel=VELOCITY, acc=ACC, radius= 20.0, ra=DR_MV_RA_DUPLICATE)                  
            #wait(0.5)
            
            # TARGET POS DOWN 위치까지 L무브
            self.get_logger().info(f"{target_pos_list} / 플레이스-다운 위치로 이동합니다. ")
            movel(posx(target_pos_list), vel=VELOCITY, acc=ACC,  radius= 0, ra=DR_MV_RA_DUPLICATE)   
            wait(0.5)

            # 물건 놓기
            self.get_logger().info("물건을 놓습니다.")
            self.gripper.move(0)
            wait(10)

            self.get_logger().info("초기 자세로 복귀합니다.")
            movej(p_start, VELOCITY, ACC)
            
            wait(0.5)
            return True
        
        
        except Exception as e:
            self.get_logger().error(f"로봇 이동 및 그리퍼 제어 중 오류 발생: {e}")
            return False
#--------------------------sys
def input_thread_fn(cmd_q: queue.Queue, busy_event: threading.Event):
    while True:
        if busy_event.is_set():
            #print(f"작업 중이라 입력 무시: {value}")
            time.sleep(1.000)
            continue
        
        s = input("오브젝트 번호 0~10 입력 (q=종료): ").strip()

        if s.lower() == 'q':
            cmd_q.put(("quit", None))
            break

        try:
            value = int(s)
        except ValueError:
            print("숫자만 입력하세요.")
            continue

        if not (0 <= value <= 10):
            print("범위 오류! 0~10만")
            continue

        if busy_event.is_set():
            print(f"작업 중이라 입력 무시: {value}")
            continue

        # 큐가 차있으면(대기중 명령 있으면) 이번 입력도 무시 (정책: 무시)
        if cmd_q.full():
            print(f"대기 중 명령이 있어 입력 무시: {value}")
            continue

        cmd_q.put(("pick_place", value))
        time.sleep(2.000)
        
#--------------------------sys


def main(args=None):
    rclpy.init(args=args)

    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    try:
        from DSR_ROBOT2 import get_current_posx, movel, wait, movej
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"DSR_ROBOT2 라이브러리를 임포트할 수 없습니다: {e}")
        rclpy.shutdown()
        exit(1)

    try:
        robot_controller = RobotControllerNode()
    except Exception as e:
        print(f"RobotControllerNode 초기화 실패: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        return

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(robot_controller)
    executor.add_node(dsr_node)

    # sys / 사용자 입력대기 (0~10) - 필요 시 사용
    cmd_q = queue.Queue(maxsize=1)
    busy_event = threading.Event()
    t_in = threading.Thread(target=input_thread_fn, args=(cmd_q, busy_event), daemon=True)
    t_in.start()

    try:
        while rclpy.ok():
            # 콜백 처리
            executor.spin_once(timeout_sec=0.01)

            now = robot_controller.get_clock().now()
            if robot_controller.selection_consumed and robot_controller.last_selected_msg_time is not None:
                elapsed_ns = (now - robot_controller.last_selected_msg_time).nanoseconds
                if elapsed_ns > int(robot_controller.selection_reset_sec * 1e9):
                    robot_controller.selection_consumed = False

            if robot_controller.new_selection_event.is_set():
                robot_controller.new_selection_event.clear()
                if busy_event.is_set() or robot_controller.selection_consumed:
                    continue

                sp = robot_controller.latest_selected_point
                if sp is None:
                    continue

                busy_event.set()
                robot_controller.selection_consumed = True
                try:
                    final_x, final_y, final_z = camera_point_to_robot_mm(sp)

                    robot_controller.pick_move_robot_and_control_gripper(
                        final_x, final_y, final_z, robot_controller.latest_selected_width, None
                    )
                    robot_controller.place_move_robot_and_control_gripper(
                        PLACE_X_MM, PLACE_Y_MM, PLACE_Z_MM, robot_controller.latest_selected_width, None
                    )
                finally:
                    busy_event.clear()

            # 입력(명령) 확인
            try:
                cmd, val = cmd_q.get_nowait()
            except queue.Empty:
                continue

            # 종료
            if cmd == "quit":
                break

            # pick/place
            if cmd == "pick_place":
                if busy_event.is_set():
                    print(f"작업 중이라 입력 무시: {val}")
                    continue

                busy_event.set()
                try:
                    value = val

                    # 픽
                    final_x = 500
                    final_y = 0 + value * 20
                    final_z = 200
                    robot_controller.pick_move_robot_and_control_gripper(final_x, final_y, final_z)

                    # 플레이스
                    final_x = 300
                    final_y = 0 + value * 20
                    final_z = 200
                    robot_controller.place_move_robot_and_control_gripper(final_x, final_y, final_z)
                finally:
                   busy_event.clear()
   

        
    except KeyboardInterrupt:
        print("Ctrl+C로 종료합니다...")

    finally:
        print("프로그램을 종료합니다...")

        try:
            if rclpy.ok():
                robot_controller.terminate_gripper()
                # 응답 처리 기회 주기
                for _ in range(50):
                    executor.spin_once(timeout_sec=0.01)
            else:
                print("ROS 컨텍스트가 이미 종료되어 그리퍼 terminate를 생략합니다.")
        except Exception as e:
            print(f"그리퍼 terminate 중 예외(종료 과정에서 흔함): {e}")

        try:
            robot_controller.destroy_node()
        except Exception:
            pass

        try:
            dsr_node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

        print("종료 완료.")



if __name__ == '__main__':
    main()
