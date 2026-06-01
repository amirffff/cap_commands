"""
Example: Multi-axis motor control with binary packets.

This demonstrates how to control multiple motors (axes) separately
using binary packet protocol.
"""
import time
from motor_driver import Controller, MotorControl
# Create driver instance (shared for all axes)

driver = Controller(host="192.168.10.120", port=4949, timeout=5.0)


def _run_get_set_pairs(motor: MotorControl) -> None:
    """Run config/IP getter-setter pairs: set(get())."""
    string_maps = {
        "get_cfg_Motor_Type": {"Stepper": 0, "BLDC": 1, "DC": 2},
        "get_cfg_Apos_Load_Type": {"Apos_load": 0, "Apos_SSI": 1, "TPOS": 2},
        "get_encoder_Location": {"None": 0, "Motor": 1, "Load": 2},
        "get_cfp_PC_COM": {"None": 0, "Ethernet": 1, "RS232": 2, "RS422": 3, "RS485": 4, "TTL": 5, "SPI": 6},
        "get_cfg_TS_COM_TYPE": {"Ethernet": 0, "RS232": 1, "CAN": 9},
        "get_cfg_Can_Baud_rate": {"None": 0, "125K": 1, "250K": 2, "500K": 3, "1M": 4},
        "get_cfg_Biss_Com": {"None": 0, "SPI1": 1, "SPI2": 2},
        "get_cfg_System_Type": {"manual": 0, "stabilized": 1, "tracker": 2, "dual_gimbal": 3},
        "get_cfg_reversed_IMU": {"client": 0, "base_imu": 1, "load_imu": 2, "mid_imu": 3},
        "get_cfg_Imu_Com": {"None": 0, "RS232_1": 1, "RS232_2": 2, "RS232_3": 3, "RS485": 4, "RS422": 5, "TTL": 6},
        # get_cfg_IMU_Type currently returns CAN baud style text in motor_driver.py.
        "get_cfg_IMU_Type": {"None": 0, "125K": 1, "250K": 2, "500K": 3, "1M": 4},
    }

    # All getter/setter pairs from configuration + IP sections.
    # Ignored by request: cfg_load, cfg_save, SaveIP.
    command_pairs = [
        ("get_cfg_Gear_Ratio", "set_cfg_Gear_Ratio"),
        ("get_cfg_Max_VDC", "set_cfg_Max_VDC"),
        ("get_cfg_Peak_Current", "set_cfg_Peak_Current"),
        ("get_cfg_slow_loop_sampling", "set_cfg_slow_loop_sampling"),
        ("get_cfg_micro_Steps", "set_cfg_micro_Steps"),
        ("get_cfg_steps_Per_Rev", "set_cfg_steps_Per_Rev"),
        ("get_cfg_close_loop_enable", "set_cfg_close_loop_enable"),
        ("get_cfg_Motor_Type", "set_cfg_Motor_Type"),
        ("get_cfg_Apos_Load_Type", "set_cfg_Apos_Load_Type"),
        ("get_cfg_Load_Encoder_Lines", "set_cfg_Load_Encoder_Lines"),
        ("get_cfg_Encoder_Lines", "set_cfg_Encoder_Lines"),
        ("get_encoder_Location", "set_encoder_Location"),
        ("get_cfp_PC_COM", "set_cfp_PC_COM"),
        ("get_cfg_TS_COM_TYPE", "set_cfg_TS_COM_TYPE"),
        ("get_cfg_Multi_Com", "set_cfg_Multi_Com"),
        ("get_cfg_Can_Baud_rate", "set_cfg_Can_Baud_rate"),
        ("get_cfg_Biss_Resolution", "set_cfg_Biss_Resolution"),
        ("get_cfg_Biss_Com", "set_cfg_Biss_Com"),
        ("get_cfg_Abs_Enc_Offset", "set_cfg_Abs_Enc_Offset"),
        ("get_cfg_Abs_Enc_Reverse", "set_cfg_Abs_Enc_Reverse"),
        ("get_cfg_Max_Speed", "set_cfg_Max_Speed"),
        ("get_cfg_Min_Speed", "set_cfg_Min_Speed"),
        ("get_cfg_max_Acceleration", "set_cfg_max_Acceleration"),
        ("get_cfg_Min_Acceleration", "set_cfg_Min_Acceleration"),
        ("get_cfg_IMU_Type", "set_cfg_IMU_TYPE"),
        ("get_cfg_reversed_IMU", "set_cfg_reversed_IMU"),
        ("get_cfg_Imu_Com", "set_cfg_Imu_Com"),
        ("get_cfg_Imu_Baud_Rate", "set_cfg_Imu_Baud_Rate"),
        ("get_cfg_Imu_Freq", "set_cfg_Imu_Freq"),
        ("get_cfg_System_Type", "set_cfg_System_Type"),
        ("get_cfg_Serial_Number", "set_cfg_Serial_Number"),
        ("get_cfg_Com_Axes_TS", "set_cfg_Com_Axes_TS"),
        ("GetControllerIP", "SetControllerIP"),
        ("GetControllerPort", "SetControllerPort"),
        ("GetControllerSubnetMask", "Set_Controller_Subnet_Mask"),
    ]

    print("\n=== Running Configuration/IP Get->Set Pairs ===")
    for get_name, set_name in command_pairs:
        getter = getattr(motor, get_name, None)
        setter = getattr(motor, set_name, None)
        if getter is None or setter is None:
            print(f"[SKIP] Missing method(s): {get_name} / {set_name}")
            continue

        try:
            value = getter()
            if value is None:
                print(f"[SKIP] {get_name} returned None")
                continue

            set_value = value
            if get_name in string_maps and isinstance(value, str):
                mapped = string_maps[get_name].get(value)
                if mapped is None:
                    print(f"[SKIP] No mapping for {get_name} value '{value}'")
                    continue
                set_value = mapped
            elif get_name == "get_cfg_Multi_Com":
                # Getter returns list of active names; setter expects a single index.
                if not value:
                    print("[SKIP] get_cfg_Multi_Com returned empty list")
                    continue
                multi_map = {"Ethernet": 0, "RS232": 1, "RS422": 2, "TTL": 3}
                first_name = value[0]
                if first_name not in multi_map:
                    print(f"[SKIP] Unknown Multi_Com name '{first_name}'")
                    continue
                set_value = multi_map[first_name]
            elif get_name in ("GetControllerIP", "GetControllerSubnetMask"):
                # Keep raw list as returned by getter.
                set_value = value

            result = setter(set_value)
            print(f"[{'OK' if result else 'FAIL'}] {set_name}({set_value})")
        except Exception as exc:
            print(f"[ERROR] {get_name} -> {set_name}: {exc}")

if driver.connect():
    print("Connected to motor driver")
    
    # Create motor control instances for different axes
    # Axis 1
    motor1 = MotorControl(
        driver=driver,
        axis_number=1,  # First axis
        max_speed=20,
        position_units="degrees"
    )
    
    # Axis 2
    motor2 = MotorControl(
        driver=driver,
        axis_number=2,  # Second axis
        max_speed=20,
        position_units="degrees"
    )
    
    print("\n=== Start Test ===")
    # Enable axis 1
    #motor1.axis_on(1)
    #motor2.axis_on(2)

    motor1.set_movement_type('relative')
    motor2.set_movement_type('relative')

    motor1.set_movement_mode('speed')
    motor2.set_movement_mode('speed')

    motor1.set_speed(0)
    motor2.set_speed(0)

    motor1.update()
    motor2.update()

    # Run all configuration and IP get/set command pairs on axis 1.
    #_run_get_set_pairs(motor1)
    motor2.get_cfg_Num_Of_Axes()
    motor2.GetControllerPort()
    motor2.GetControllerSubnetMask()
    motor2.GetControllerIP()
    
print('test finished successfully')