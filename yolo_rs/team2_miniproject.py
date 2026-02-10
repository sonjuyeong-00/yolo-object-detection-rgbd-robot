import cv2
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import time

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import message_filters

import DR_init
from yolo_rs.gripper_drl_controller import GripperController
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_point

#--------------------------sys
import threading
import queue
#--------------------------sys

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
VELOCITY, ACC = 100, 100

#--------------------------sys
# move reaction
DR_MV_RA_NONE      = 0
DR_MV_RA_DUPLICATE = 0
DR_MV_RA_OVERRIDE  = 1

GRIPPER_GRAB_POS = 300
GRIPPER_RELEASE_POS = 100

# RH-P12-RN 기준(대략): 스트로크 0~106mm, Goal Position 0~1150
GRIPPER_STROKE_MAX_MM = 106.0
GRIPPER_PULSE_MAX = 1150
GRIPPER_GRAB_MARGIN_MM = 2.0     # 물체 폭보다 조금 더 닫기
GRIPPER_RELEASE_MARGIN_MM = 10.0 # 놓을 때 더 열기
GRIPPER_PLACE_UP_EXTRA_MM = 5.0  # 플레이스 업 전 추가로 살짝 열기

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

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("robot_controller_node")

        
        self.get_logger().info("ROS 2 구독자 설정을 시작합니다.")
        
        """
        self.bridge = CvBridge()

        self.intrinsics = None
        self.latest_cv_color = None
        self.latest_cv_depth_mm = None

        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw'
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw'
            #self, Image, '/camera/camera/depth/image_rect_raw'
        )
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info'
            #self, CameraInfo, '/camera/camera/depth/camera_info'
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)
        
        
        self.get_logger().info("컬러/뎁스/카메라정보 토픽 구독 대기 중...")
        self.get_logger().info("화면이 나오지 않으면 Launch 명령어를 확인하세요.")
        """
        
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

        # TF2 buffer/listener for camera -> robot base transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = "base_link"

    def synced_callback(self, color_msg, depth_msg, info_msg):
        """
        try:
            self.latest_cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            self.latest_cv_depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge 변환 오류: {e}")
            return

        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = info_msg.width
            self.intrinsics.height = info_msg.height
            self.intrinsics.ppx = info_msg.k[2]
            self.intrinsics.ppy = info_msg.k[5]
            self.intrinsics.fx = info_msg.k[0]
            self.intrinsics.fy = info_msg.k[4]
            
            if info_msg.distortion_model == 'plumb_bob' or info_msg.distortion_model == 'rational_polynomial':
                self.intrinsics.model = rs.distortion.brown_conrady
            else:
                self.intrinsics.model = rs.distortion.none
            
            self.intrinsics.coeffs = list(info_msg.d)
            self.get_logger().info("카메라 내장 파라미터(Intrinsics) 수신 완료.")
        """

    def stop_camera(self):
        pass

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

    def mouse_callback(self, event, u, v, flags, param):
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.latest_cv_depth_mm is None or self.intrinsics is None:
                self.get_logger().warn("아직 뎁스 프레임 또는 카메라 정보가 수신되지 않았습니다.")
                return

            try:
                depth_mm = self.latest_cv_depth_mm[v, u]
            except IndexError:
                self.get_logger().warn(f"클릭 좌표(u={u}, v={v})가 이미지 범위를 벗어났습니다.")
                return
            
            if depth_mm == 0:
                print(f"({u}, {v}) 지점의 깊이를 측정할 수 없습니다 (값: 0).")
                return

            depth_m = float(depth_mm) / 1000.0

            point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_m)

            x_mm = point_3d[1] * 1000
            y_mm = point_3d[0] * 1000
            z_mm = point_3d[2] * 1000

            final_x = 620 + x_mm
            final_y = y_mm
            final_z = 950 - z_mm
            if(final_z <= 110):
                final_z = 110

            print("--- 변환된 최종 3D 좌표 ---")
            print(f"픽셀 좌표: (u={u}, v={v}), Depth: {depth_m*1000:.1f} mm")
            print(f"로봇 목표 좌표: X={final_x:.1f}, Y={final_y:.1f}, Z={final_z:.1f}\n")

            self.pick_move_robot_and_control_gripper(final_x, final_y, final_z)
            print("=" * 50)
        """

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
            self.gripper.move(0)
            
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

   
    
    # OpenCV 카메라 화면 생성 및 마우스 콜백처리 
    """
    cv2.namedWindow("RealSense Camera")
    cv2.setMouseCallback("RealSense Camera", robot_controller.mouse_callback)
    print("카메라 영상에서 원하는 지점을 클릭하세요. 'ESC' 키를 누르면 종료됩니다.")
    """
    
    # sys / 사용자 입력대기 (0~10) - 필요 시 사용
    cmd_q = queue.Queue(maxsize=1)
    busy_event = threading.Event()
    t_in = threading.Thread(target=input_thread_fn, args=(cmd_q, busy_event), daemon=True)
    t_in.start()

    busy = False
    try:
        while rclpy.ok():
            # 콜백 처리
            rclpy.spin_once(robot_controller, timeout_sec=0.001)
            rclpy.spin_once(dsr_node, timeout_sec=0.001)

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
                    # TF로 camera frame -> base_link 변환 (m -> mm)
                    try:
                        tfm = robot_controller.tf_buffer.lookup_transform(
                            robot_controller.target_frame,
                            sp.header.frame_id,
                            sp.header.stamp,
                            timeout=Duration(seconds=0.5),
                        )
                        sp_base = do_transform_point(sp, tfm)
                        x_mm = sp_base.point.x * 1000.0
                        y_mm = sp_base.point.y * 1000.0
                        z_mm = sp_base.point.z * 1000.0
                        final_x, final_y, final_z = x_mm, y_mm, z_mm
                        robot_controller.get_logger().info(
                            f"TF 변환 OK: {sp.header.frame_id} -> {robot_controller.target_frame} | "
                            f"XYZ(mm)=({final_x:.1f}, {final_y:.1f}, {final_z:.1f})"
                        )
                    except (LookupException, ConnectivityException, ExtrapolationException) as e:
                        # TF 실패 시 기존 오프셋 방식으로 fallback
                        robot_controller.get_logger().warn(
                            f"TF 변환 실패({e}); 오프셋 보정으로 fallback 합니다."
                        )
                        x_mm = sp.point.x * 1000.0
                        y_mm = sp.point.y * 1000.0
                        z_mm = sp.point.z * 1000.0
                        # 카메라->로봇 축 매핑: X<-Y, Y<-X, Z<- -Z
                        final_x = CAM_TO_ROBOT_OFFSET_X_MM + (CAM_TO_ROBOT_SIGN_X * y_mm)
                        final_y = CAM_TO_ROBOT_OFFSET_Y_MM + (CAM_TO_ROBOT_SIGN_Y * x_mm)
                        final_z = CAM_TO_ROBOT_OFFSET_Z_MM + (CAM_TO_ROBOT_SIGN_Z * z_mm)

                    if final_z <= 110.0:
                        final_z = 110.0

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
                time.sleep(0.001)
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
                    rclpy.spin_once(robot_controller, timeout_sec=0.01)
                    rclpy.spin_once(dsr_node, timeout_sec=0.01)
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
