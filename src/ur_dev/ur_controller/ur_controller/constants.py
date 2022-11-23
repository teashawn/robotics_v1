from ur_controller import models
from visualization_msgs.msg import MarkerArray, Marker
from math import radians

GRIPPER_HEADER = """
def Gripper():
  set_tcp(p[0.0,0.0,0.248,0.0,0.0,0.0])
  set_payload(1.0,[-3.0E-4,4.0E-4,0.0785])
  step_count_c1ea570d_597a_4e27_b1f2_f1204aefbef2 = 0
  thread Step_Counter_Thread_c0f00319_af92_46b1_ade4_4c060a86bf18():
    while (True):
      step_count_c1ea570d_597a_4e27_b1f2_f1204aefbef2 = step_count_c1ea570d_597a_4e27_b1f2_f1204aefbef2 + 1
      sync()
    end
  end
  run Step_Counter_Thread_c0f00319_af92_46b1_ade4_4c060a86bf18()
  set_tool_communication(True, 115200, 0, 1, 1.5, 3.5)
  set_tool_output_mode(0)
  set_tool_digital_output_mode(0, 1)
  set_tool_digital_output_mode(1, 1)
  set_tool_voltage(24)
  set_safety_mode_transition_hardness(1)
  set_gravity([0.0, 0.0, 9.82])
  set_standard_analog_input_domain(0, 1)
  set_standard_analog_input_domain(1, 1)
  set_tool_analog_input_domain(0, 1)
  set_tool_analog_input_domain(1, 1)
  set_analog_outputdomain(0, 0)
  set_analog_outputdomain(1, 0)
  set_input_actions_to_default()
  global Tag_close=p[0.2500499835531529,-0.410935418095572,-0.01113966830128063,-1.7874951184609469,-1.8017232492820372,-0.7402163074892504]
  global Tag_far=p[0.43738918447632236,-0.7281082466167076,0.20760428828474234,-0.24290281172126074,-2.2958250918923513,0.5862200531861109]
  global big_bottle_top=p[0.6099959107815354,-5.170117678572837E-5,0.4590314220131131,-1.929901603467692,1.9223572188918348,-0.5187005855228849]
  global calib_tablette=p[0.43742438444231707,-0.7280888776896464,0.20760071381319223,-0.24283482165419867,-2.295834628749297,0.5862504240956105]
  global calibration=p[0.6099918989958789,-4.2730767611359823E-5,0.2899891861580802,-1.9299399457374553,1.9223242782383454,-0.5187170534687505]
  global contact_offset=p[0.5831983576595184,-0.1690064271814607,0.30217642918791476,-1.939084111827569,1.9109989560326721,-0.5153981880903973]
  global workshop_far_t=p[0.20524282924776238,-0.6510860678627466,0.31617165786262214,-0.32063819509492913,-2.237715686465029,0.8284569749956258]
  # begin: URCap Installation Node
  #   Source: Robotiq_Grippers, 1.8.0.1, Robotiq Inc.
  #   Type: Vacuum
  #################################################
  # Vacuum Grip Check
  #################################################
  
  vacuumGripCheckThread1 = 0
  vacuumGripCheckThread2 = 0
  vacuumGripCheckThread3 = 0
  vacuumGripCheckThread4 = 0
  vacuumGripCheckThread1Running = False
  vacuumGripCheckThread2Running = False
  vacuumGripCheckThread3Running = False
  vacuumGripCheckThread4Running = False
  vacuumGripCheckSocketId = "1"
  vacuumGripCheckThreadStarted = False
  
  thread vacuumGripCheck():
    gripper_socket = vacuumGripCheckSocketId
    vacuumGripCheckThreadStarted = True
  
    while (True):
      objectDetectedDebounceCtr = 0
  
      while (objectDetectedDebounceCtr < 3):
        if(rq_is_vacuum_obj_detected(gripper_socket="1")):
          objectDetectedDebounceCtr = objectDetectedDebounceCtr + 1
        else:
          objectDetectedDebounceCtr = 0
        end
        sleep(0.1)
      end
  
      objectNotDetectedDebounceCtr = 0
  
      while objectNotDetectedDebounceCtr < 3:
        if (not rq_is_vacuum_obj_detected(gripper_socket="1")):
          objectNotDetectedDebounceCtr = objectNotDetectedDebounceCtr + 1
        else:
          objectNotDetectedDebounceCtr = 0
        end
        sleep(0.1)
      end
  
      socket_open("127.0.0.1",29999,"dashboardServerSocket")
      socket_send_line("pause","dashboardServerSocket")
      socket_send_string("popup Vacuum grip check has detected an object drop.", "dashboardServerSocket")
      socket_send_byte(10, "dashboardServerSocket")
      socket_close("dashboardServerSocket")
  
      sync()
    end
  end
  
  def startVacuumGripCheckThread(gripperId="1"):
    vacuumGripCheckSocketId = gripperId
    threadHandle = run vacuumGripCheck()
    waitForVacuumGripCheckThreadStarted()
    return threadHandle
  end
  
  def stopVacuumGripCheckThread(threadHandle):
    kill threadHandle
  end
  
  def waitForVacuumGripCheckThreadStarted():
    while (not(vacuumGripCheckThreadStarted)):
      sync()
    end
    vacuumGripCheckThreadStarted = False
  end
  #################################################
  # End - Vacuum Grip Check
  #################################################
  
  #################################################
  # Stops the pump on a distance travelled
  #################################################
  global stopPumpDistance = 100
  global stopPumpSocketId = "0"
  global stopPumpThreadStarted = [False, False, False, False]
  global stopPumpThreadHandles = [0, 0, 0, 0]
  
  thread stopPumpOnDistanceTravelled():
    distance = stopPumpDistance
    socketId = stopPumpSocketId
    stopPumpThreadStarted[rq_socket_to_index(socketId)] = True
  
    measuredDistance = waitForDistanceTravelled(distance)
  
    rq_stop(socketId)
  
    stopPumpThreadStarted[rq_socket_to_index(socketId)] = False
  end
  
  def waitForDistanceTravelled(distance):
    startingPose = get_actual_tcp_pose()
    measuredDistance = 0
    while (measuredDistance < distance):
      sleep(0.1)
      measuredDistance = point_dist(get_actual_tcp_pose(), startingPose)
    end
  
    return measuredDistance
  end
  
  def startStopPumpOnDistanceTravelledThread(distance, gripper_socket="1"):
    if (stopPumpThreadStarted[rq_socket_to_index(gripper_socket)]):
      return 0
    end
  
    global stopPumpDistance = distance
    global stopPumpSocketId = gripper_socket
    stopPumpThreadHandles[rq_socket_to_index(gripper_socket)] = run stopPumpOnDistanceTravelled()
    waitForStopPumpOnDistanceTravelledThreadStarted(gripper_socket)
    return stopPumpThreadHandles[rq_socket_to_index(gripper_socket)]
  end
  
  def waitForStopPumpOnDistanceTravelledThreadStarted(gripper_socket="1"):
    while (not(stopPumpThreadStarted[rq_socket_to_index(gripper_socket)])):
      sync()
    end
  end
  
  def stopStopPumpOnDistanceTravelledThread(gripper_socket="1"):
    handle = stopPumpThreadHandles[rq_socket_to_index(gripper_socket)]
    threadIsRunning = stopPumpThreadStarted[rq_socket_to_index(gripper_socket)]
    if (threadIsRunning):
      kill handle
      clear_socket_buffer(gripper_socket, 0.01)
      stopPumpThreadHandles[rq_socket_to_index(gripper_socket)] = 0
    end
  end
  #################################################
  # End - Stops the pump on a distance travelled
  #################################################
  
  #################################################
  # Vacuum general functions
  #################################################
  def rq_wait_for_vacuum_object_detected(gripper_socket="1"):
      while (not rq_is_vacuum_obj_detected(gripper_socket)):
          if (rq_is_vacuum_timeout(gripper_socket)):
              return False
          end
          sync()
      end
      return True
  end
  
  def rq_wait_for_vacuum_object_secured(gripper_socket="1"):
      while (not rq_is_vacuum_obj_secured(gripper_socket)):
          if (rq_is_vacuum_timeout(gripper_socket)):
              return False
          end
          sync()
      end
      return True
  end
  
  def rq_wait_for_vacuum_object_suction_complete(gripper_socket="1"):
      remaining_retries = 5
  
       # Wait for suction started
      while (not rq_is_vacuum_obj_in_suction(gripper_socket) and
             not rq_is_vacuum_obj_detected(gripper_socket) and
             remaining_retries > 0):
          sleep(0.01)
          remaining_retries = remaining_retries - 1
      end
  
      # Wait for suction completed
      while (rq_is_vacuum_obj_in_suction(gripper_socket)):
          if (rq_is_vacuum_timeout(gripper_socket)):
              return False
          end
          sleep(0.01)
      end
  
      return True
  end
  
  def rq_wait_for_vacuum_object_not_detected(gripper_socket="1"):
      while (rq_is_vacuum_obj_detected(gripper_socket)):
          sleep(0.01)
      end
  end
  
  def rq_is_vacuum_obj_detected(gripper_socket="1"):
      gOBJ = rq_get_var("OBJ", 1, gripper_socket)
      sync()
      return is_vacuum_OBJ_object_detected(gOBJ)
  end
  
  def rq_is_vacuum_obj_secured(gripper_socket="1"):
      gOBJ = rq_get_var("OBJ", 1, gripper_socket)
      sync()
      return is_vacuum_OBJ_object_secured(gOBJ)
  end
  
  def rq_is_vacuum_obj_in_suction(gripper_socket="1"):
      gOBJ = rq_get_var("OBJ", 1, gripper_socket)
      sync()
  
      if(is_vacuum_OBJ_object_in_motion(gOBJ)):
          return True
      else:
          return False
      end
  end
  
  def rq_is_vacuum_timeout(gripper_socket="1"):
      gFLT = rq_get_var("FLT", 2, gripper_socket)
      sync()
  
      if(gFLT ==6):
          return True
      end
  
      return False
  end
  
  def is_vacuum_OBJ_object_in_motion(gOBJ):
      if (gOBJ == 0):
          return True
      end
  
      return False
  end
  
  def is_vacuum_OBJ_object_detected(gOBJ):
      if (gOBJ == 1 or gOBJ == 2):
          return True
      end
  
      return False
  end
  
  def is_vacuum_OBJ_object_secured(gOBJ):
      if (gOBJ == 2):
          return True
      end
  
      return False
  end
  
  def rq_set_pressure_timeout_minimum_vacuum(pressure, timeout, minimum, gripper_socket="1"):
      rq_set_pos_spd_for(pressure, timeout, minimum, gripper_socket)
  end
  
  def is_FLT_vacuum_timeout(gFLT):
    if (gFLT == 6):
        return True
    end
  
    return False
  end
  
  def is_continuous_grip(maximum_vacuum):
    return maximum_vacuum == 0
  end
  
  def rq_vacuum_release(advanced_mode=False, shutoff_distance_cm=5, wait_for_object_released=True, gripper_socket="1"):
    local shutoff_distance = scale(shutoff_distance_cm, [0, 99], [0.00, 0.99])
    local pressure = 255
    local minimum = 0
    local timeout = 255
    rq_vacuum_release_raw(advanced_mode, pressure, minimum, timeout, shutoff_distance, wait_for_object_released, gripper_socket)
  end
  
  def rq_vacuum_release_raw(advanced_mode, pressure, minimum, timeout, shutoff_distance, wait_for_object_released, gripper_socket):
    rq_reset_fault_and_activate(gripper_socket)
    rq_set_pressure_timeout_minimum_vacuum(pressure, timeout, minimum, gripper_socket)
  
    if advanced_mode:
      rq_set_gripper_mode(1, gripper_socket)
    else:
      rq_set_gripper_mode(0, gripper_socket)
    end
  
    rq_set_GTO_and_wait(1, gripper_socket)
  
    if wait_for_object_released:
      rq_wait_for_vacuum_object_not_detected(gripper_socket)
    end
  
    if advanced_mode:
      startStopPumpOnDistanceTravelledThread(shutoff_distance, gripper_socket)
    end
  end
  
  def rq_vacuum_grip(advanced_mode=False, maximum_vacuum=60, minimum_vacuum=40, timeout_ms=3000, wait_for_object_detected=True, gripper_socket="1"):
    local pressure = scale(maximum_vacuum, [0, 100], [100, 0])
    local minimum = scale(minimum_vacuum, [0, 100], [100, 0])
    local timeout = scale(timeout_ms, [0, 25500], [0, 255])
    rq_vacuum_grip_raw(advanced_mode, pressure, minimum, timeout, gripper_socket)
    if wait_for_object_detected:
          suction_completed = rq_wait_for_vacuum_object_suction_complete(gripper_socket)
          if(not suction_completed):
              rq_set_var("GTO", 0, gripper_socket)
          end
    end
  end
  
  def rq_vacuum_grip_raw(advanced_mode, pressure, minimum, timeout, gripper_socket):
    stopStopPumpOnDistanceTravelledThread(gripper_socket)
    rq_reset_fault_and_activate(gripper_socket)
  
    rq_set_pressure_timeout_minimum_vacuum(pressure, timeout, minimum, gripper_socket)
  
    if advanced_mode:
      rq_set_gripper_mode(1, gripper_socket)
    else:
      rq_set_gripper_mode(0, gripper_socket)
    end
  
    rq_set_GTO_and_wait(1, gripper_socket)
  
  end
  
  def rq_reset_fault_and_activate(gripper_socket):
      gFLT = rq_get_var("FLT", 2, gripper_socket)
  
      if(not is_FLT_no_fault(gFLT)):
          if(is_FLT_vacuum_timeout(gFLT)):
              rq_set_GTO_and_wait(0, gripper_socket)
          elif(is_FLT_faulted(gFLT)):
              rq_set_GTO_and_wait(0, gripper_socket)
              rq_set_var("ACT", 1, gripper_socket)
          end
      elif(not rq_is_gripper_activated(gripper_socket)):
          rq_set_GTO_and_wait(0, gripper_socket)
          rq_set_var("ACT", 1, gripper_socket)
      end
  end
  #################################################
  # End - Vacuum general functions
  #################################################
  vacuumGripCheckWarningTitle = "Vacuum grip check"
  vacuumGripCheckWarningMessage = "Vacuum gripper object lost"
  vacuumGripTimeoutTitle = "Vacuum gripper fault"
  vacuumGripTimeoutMessage = "Grip has timed out"
  # end: URCap Installation Node
  # begin: URCap Installation Node
  #   Source: Robotiq_Grippers, 1.8.0.1, Robotiq Inc.
  #   Type: Gripper
  rq_gripper_socket_ip_address = "127.0.0.1"
  rq_gripper_socket_port = 63352
  
  rq_comm_check_fail_counter = [0, 0, 0, 0]
  
  rq_read_act = [-1, -1, -1, -1]
  rq_read_gto = [-1, -1, -1, -1]
  rq_read_for = [-1, -1, -1, -1]
  rq_read_spe = [-1, -1, -1, -1]
  rq_read_obj = [-1, -1, -1, -1]
  rq_read_sta = [-1, -1, -1, -1]
  rq_read_flt = [-1, -1, -1, -1]
  rq_read_pos = [-1, -1, -1, -1]
  rq_read_pre = [-1, -1, -1, -1]
  rq_read_lbp = [-1, -1, -1, -1]
  rq_read_lrd = [-1, -1, -1, -1]
  rq_read_lbl = [-1, -1, -1, -1]
  rq_read_lgn = [-1, -1, -1, -1]
  rq_read_msc = [-1, -1, -1, -1]
  rq_read_mod = [-1, -1, -1, -1]
  rq_read_cou = [-1, -1, -1, -1]
  rq_read_ncy = [-1, -1, -1, -1]
  rq_read_dst = [-1, -1, -1, -1]
  rq_read_pco = [-1, -1, -1, -1]
  
  rq_string_initial_value = "N/A"
  
  rq_read_snu_1 = rq_string_initial_value
  rq_read_snu_2 = rq_string_initial_value
  rq_read_snu_3 = rq_string_initial_value
  rq_read_snu_4 = rq_string_initial_value
  
  rq_read_fwv_1 = rq_string_initial_value
  rq_read_fwv_2 = rq_string_initial_value
  rq_read_fwv_3 = rq_string_initial_value
  rq_read_fwv_4 = rq_string_initial_value
  
  rq_read_ver_1 = rq_string_initial_value
  rq_read_ver_2 = rq_string_initial_value
  rq_read_ver_3 = rq_string_initial_value
  rq_read_ver_4 = rq_string_initial_value
  
  rq_read_act_req = [True, True, True, True]
  rq_read_gto_req = [True, True, True, True]
  rq_read_for_req = [True, True, True, True]
  rq_read_spe_req = [True, True, True, True]
  rq_read_obj_req = [True, True, True, True]
  rq_read_sta_req = [True, True, True, True]
  rq_read_flt_req = [True, True, True, True]
  rq_read_pos_req = [True, True, True, True]
  rq_read_pre_req = [True, True, True, True]
  rq_read_lbp_req = [True, True, True, True]
  rq_read_lrd_req = [True, True, True, True]
  rq_read_lbl_req = [True, True, True, True]
  rq_read_lgn_req = [True, True, True, True]
  rq_read_msc_req = [True, True, True, True]
  rq_read_mod_req = [True, True, True, True]
  rq_read_cou_req = [True, True, True, True]
  rq_read_ncy_req = [True, True, True, True]
  rq_read_dst_req = [True, True, True, True]
  rq_read_pco_req = [True, True, True, True]
  
  rq_read_snu_1_req = False
  rq_read_snu_2_req = False
  rq_read_snu_3_req = False
  rq_read_snu_4_req = False
  
  rq_read_fwv_1_req = False
  rq_read_fwv_2_req = False
  rq_read_fwv_3_req = False
  rq_read_fwv_4_req = False
  
  rq_read_ver_1_req = False
  rq_read_ver_2_req = False
  rq_read_ver_3_req = False
  rq_read_ver_4_req = False
  
  rq_write_act_request = [False, False, False, False]
  rq_write_gto_request = [False, False, False, False]
  rq_write_atr_request = [False, False, False, False]
  rq_write_ard_request = [False, False, False, False]
  rq_write_pos_request = [False, False, False, False]
  rq_write_lbp_request = [False, False, False, False]
  rq_write_lrd_request = [False, False, False, False]
  rq_write_lbl_request = [False, False, False, False]
  rq_write_lgn_request = [False, False, False, False]
  rq_write_msc_request = [False, False, False, False]
  rq_write_mod_request = [False, False, False, False]
  
  rq_write_act = [-1, -1, -1, -1]
  rq_write_gto = [-1, -1, -1, -1]
  rq_write_atr = [-1, -1, -1, -1]
  rq_write_ard = [-1, -1, -1, -1]
  rq_write_for = [-1, -1, -1, -1]
  rq_write_spe = [-1, -1, -1, -1]
  rq_write_pos = [-1, -1, -1, -1]
  rq_write_lbp = [-1, -1, -1, -1]
  rq_write_lrd = [-1, -1, -1, -1]
  rq_write_lbl = [-1, -1, -1, -1]
  rq_write_lgn = [-1, -1, -1, -1]
  rq_write_msc = [-1, -1, -1, -1]
  rq_write_mod = [-1, -1, -1, -1]
  
  rq_write_act_previous = rq_write_act
  rq_write_gto_previous = rq_write_gto
  rq_write_atr_previous = rq_write_atr
  rq_write_ard_previous = rq_write_ard
  rq_write_for_previous = rq_write_for
  rq_write_spe_previous = rq_write_spe
  rq_write_pos_previous = rq_write_pos
  rq_write_lbp_previous = rq_write_lbp
  rq_write_lrd_previous = rq_write_lrd
  rq_write_lbl_previous = rq_write_lbl
  rq_write_lgn_previous = rq_write_lgn
  rq_write_msc_previous = rq_write_msc
  rq_write_mod_previous = rq_write_mod
  
  gripper_connected = [False, False, False, False]
  gripper_socket_open = [False, False, False, False]
  rq_comm_clear_socket_buffer_enabled = [True, True, True, True]
  rq_comm_check_counter = 0
  rq_gripper_communication_thread_started = False
  
  thread rq_gripper_communication():
      rq_comm_read_constants()
      rq_comm_read_variables()
      rq_comm_initialize_write_values()
  
      while(True):
          rq_comm_clear_socket_buffer()
          rq_comm_check()
          rq_comm_read_variables()
          rq_comm_write_variables()
  
          rq_gripper_communication_thread_started = True
          sync()
      end
  end
  
  def rq_socket_to_index(gripper_socket="1"):
      # Patch in case gripper_socket is an integer
      gripper_socket_string = str_cat("", gripper_socket)
  
      if(gripper_socket_string == "1"):
          return 0
      elif(gripper_socket_string == "2"):
          return 1
      elif(gripper_socket_string == "3"):
          return 2
      elif(gripper_socket_string == "4"):
          return 3
      end
      return 0
  end
  
  def rq_index_to_socket(index=0):
      if(index == 0):
          return "1"
      elif(index == 1):
          return "2"
      elif(index == 2):
          return "3"
      elif(index == 3):
          return "4"
      end
      return "1"
  end
  
  def rq_comm_check():
      index = 0
      rq_comm_check_fail_counter_max = 99999
  
      if(rq_comm_check_counter < 50):
          rq_comm_check_counter = rq_comm_check_counter + 1
      else:
          rq_comm_check_counter = 0
  
          while(index <= 3):
              socket = rq_index_to_socket(index)
  
              if(gripper_connected[index]):
                  # Patch in case gripper_socket is an integer
                  gripper_socket_string = str_cat("", socket)
  
                  sid_list = rq_get_sid(socket)
                  is_gripper_in_sid_list = rq_is_gripper_in_sid_list(gripper_socket_string, sid_list)
  
                  if(is_gripper_in_sid_list):
                      rq_comm_check_fail_counter[index] = 0
                  else:
                      rq_comm_check_fail_counter[index] = rq_comm_check_fail_counter[index] + 1
                  end
  
                  if(rq_comm_check_fail_counter[index] > rq_comm_check_fail_counter_max):
                      message = str_cat("Communication lost with Robotiq's Gripper Slave ID ", gripper_socket_string)
                      popup(message, "Communication Error", False, True, True)
                  end
              end
              index = index + 1
          end
      end
  end
  
  def rq_init_comm_if_connected(gripper_sid=9, gripper_socket="1"):
      if(not is_gripper_socket_open(gripper_socket)):
        open_gripper_socket(gripper_socket)
      end
  
      socket_sid_set = rq_set_sid(gripper_sid, gripper_socket)
  
      if(socket_sid_set):
          # Patch in case gripper_socket is an integer
          gripper_socket_string = str_cat("", gripper_socket)
  
          sid_list = rq_get_sid(gripper_socket)
          is_gripper_in_sid_list = rq_is_gripper_in_sid_list(gripper_socket_string, sid_list)
  
          if(is_gripper_in_sid_list):
              rq_set_gripper_connected(gripper_socket_string)
              return True
          end
      end
  
      return False
  end
  
  def open_gripper_socket(gripper_socket="1"):
      is_open = socket_open(rq_gripper_socket_ip_address, rq_gripper_socket_port, gripper_socket)
      set_gripper_socket_open(gripper_socket, is_open)
  end
  
  def is_gripper_socket_open(gripper_socket="1"):
      return gripper_socket_open[rq_socket_to_index(gripper_socket)]
  end
  
  def set_gripper_socket_open(gripper_socket, is_open):
      gripper_socket_open[rq_socket_to_index(gripper_socket)] = is_open
  end
  
  def rq_set_gripper_connected(gripper_id="1"):
      gripper_connected[rq_socket_to_index(gripper_id)] = True
  end
  
  def rq_is_gripper_connected(gripper_id="1"):
      return gripper_connected[rq_socket_to_index(gripper_id)]
  end
  
  def rq_set_sid(gripper_sid=9, gripper_socket="1"):
      socket_set_var("SID", gripper_sid,  gripper_socket)
      ack = socket_read_byte_list(3, gripper_socket)
      return is_ack(ack)
  end
  
  def rq_get_sid(gripper_socket="1"):
      socket_send_string("GET SID", gripper_socket)
      sid_list = socket_read_byte_list(17, gripper_socket)
      return sid_list
  end
  
  def rq_wait_for_gripper_connected():
      gripper_socket = "gripper_conn_socket"
      socket_open(rq_gripper_socket_ip_address, rq_gripper_socket_port, gripper_socket)
  
      remainingRetries = 2000
      sid_list = rq_get_sid(gripper_socket)
      gripper_is_connected = rq_is_any_gripper_connected(sid_list)
  
      while(not gripper_is_connected and remainingRetries > 0):
          remainingRetries = remainingRetries - 1
          sid_list = rq_get_sid(gripper_socket)
          gripper_is_connected = rq_is_any_gripper_connected(sid_list)
      end
  
      socket_close(gripper_socket)
  end
  
  def rq_is_any_gripper_connected(sid_list):
      is_gripper_1_connected = rq_is_gripper1_in_sid_list(sid_list)
      is_gripper_2_connected = rq_is_gripper2_in_sid_list(sid_list)
      is_gripper_3_connected = rq_is_gripper3_in_sid_list(sid_list)
      is_gripper_4_connected = rq_is_gripper4_in_sid_list(sid_list)
  
      return is_gripper_1_connected or is_gripper_2_connected or is_gripper_3_connected or is_gripper_4_connected
  end
  
  def rq_is_gripper_ascii_in_sid_list(gripper_ascii_sid, sid_list):
      sid_list_length = sid_list[0]
      sid_list_empty_length = 2
  
      if (sid_list_length <= sid_list_empty_length):
          return False
      end
  
      sid1 = sid_list[2]
      sid2 = sid_list[5]
      sid3 = sid_list[8]
      sid4 = sid_list[11]
  
      return sid1 == gripper_ascii_sid or sid2 == gripper_ascii_sid or sid3 == gripper_ascii_sid or sid4 == gripper_ascii_sid
  end
  
  def rq_is_gripper_in_sid_list(gripper_socket_string, sid_list):
      if(gripper_socket_string == "1"):
          return rq_is_gripper1_in_sid_list(sid_list)
      elif(gripper_socket_string == "2"):
          return rq_is_gripper2_in_sid_list(sid_list)
      elif(gripper_socket_string == "3"):
          return rq_is_gripper3_in_sid_list(sid_list)
      elif(gripper_socket_string == "4"):
          return rq_is_gripper4_in_sid_list(sid_list)
      end
      return False
  end
  
  def rq_is_gripper1_in_sid_list(sid_list):
      gripper_1_sid_ascii = 57
      return rq_is_gripper_ascii_in_sid_list(gripper_1_sid_ascii, sid_list)
  end
  
  def rq_is_gripper2_in_sid_list(sid_list):
      gripper_2_sid_ascii = 50
      return rq_is_gripper_ascii_in_sid_list(gripper_2_sid_ascii, sid_list)
  end
  
  def rq_is_gripper3_in_sid_list(sid_list):
      gripper_3_sid_ascii = 51
      return rq_is_gripper_ascii_in_sid_list(gripper_3_sid_ascii, sid_list)
  end
  
  def rq_is_gripper4_in_sid_list(sid_list):
      gripper_4_sid_ascii = 52
      return rq_is_gripper_ascii_in_sid_list(gripper_4_sid_ascii, sid_list)
  end
  
  def rq_comm_clear_socket_buffer():
      index = 0
  
      while(index <= 3):
          socket = rq_index_to_socket(index)
  
          if(gripper_connected[index] and rq_comm_clear_socket_buffer_enabled[index]):
              byte_in_buffer = socket_read_byte_list(1, socket, 0.002)
              while(byte_in_buffer[0] >= 1):
                  byte_in_buffer = socket_read_byte_list(1, socket, 0.002)
              end
              rq_comm_clear_socket_buffer_enabled[index] = False
          end
          index = index + 1
      end
  end
  
  def rq_comm_initialize_write_values():
      index = 0
  
      while(index <= 3):
          socket = rq_index_to_socket(index)
  
          if(gripper_connected[index]):
  
              if(rq_write_act[index] == -1 and rq_read_act[index] != -1):
                  rq_write_act[index] = rq_read_act[index]
                  rq_write_act_previous[index] = rq_read_act[index]
              elif(rq_write_act_previous[index] == -1 and rq_read_act[index] != -1):
                  rq_write_act_previous[index] = rq_read_act[index]
              end
  
              if(rq_write_gto[index] == -1 and rq_read_gto[index] != -1):
                  rq_write_gto[index] = rq_read_gto[index]
                  rq_write_gto_previous[index] = rq_read_gto[index]
              elif(rq_write_gto_previous[index] == -1 and rq_read_gto[index] != -1):
                  rq_write_gto_previous[index] = rq_read_gto[index]
              end
  
              if(rq_write_for[index] == -1 and rq_read_for[index] != -1):
                  rq_write_for[index] = rq_read_for[index]
                  rq_write_for_previous[index] = rq_read_for[index]
              elif(rq_write_for_previous[index] == -1 and rq_read_for[index] != -1):
                  rq_write_for_previous[index] = rq_read_for[index]
              end
  
              if(rq_write_spe[index] == -1 and rq_read_spe[index] != -1):
                  rq_write_spe[index] = rq_read_spe[index]
                  rq_write_spe_previous[index] = rq_read_spe[index]
              elif(rq_write_spe_previous[index] == -1 and rq_read_spe[index] != -1):
                  rq_write_spe_previous[index] = rq_read_spe[index]
              end
  
              if(rq_write_pos[index] == -1 and rq_read_pos[index] != -1):
                  rq_write_pos[index] = rq_read_pos[index]
                  rq_write_pos_previous[index] = rq_read_pos[index]
              elif(rq_write_pos_previous[index] == -1 and rq_read_pos[index] != -1):
                  rq_write_pos_previous[index] = rq_read_pos[index]
              end
  
              if(rq_write_lbp[index] == -1 and rq_read_lbp[index] != -1):
                  rq_write_lbp[index] = rq_read_lbp[index]
                  rq_write_lbp_previous[index] = rq_read_lbp[index]
              elif(rq_write_lbp_previous[index] == -1 and rq_read_lbp[index] != -1):
                  rq_write_lbp_previous[index] = rq_read_lbp[index]
              end
  
              if(rq_write_lrd[index] == -1 and rq_read_lrd[index] != -1):
                  rq_write_lrd[index] = rq_read_lrd[index]
                  rq_write_lrd_previous[index] = rq_read_lrd[index]
              elif(rq_write_lrd_previous[index] == -1 and rq_read_lrd[index] != -1):
                  rq_write_lrd_previous[index] = rq_read_lrd[index]
              end
  
              if(rq_write_lbl[index] == -1 and rq_read_lbl[index] != -1):
                  rq_write_lbl[index] = rq_read_lbl[index]
                  rq_write_lbl_previous[index] = rq_read_lbl[index]
              elif(rq_write_lbl_previous[index] == -1 and rq_read_lbl[index] != -1):
                  rq_write_lbl_previous[index] = rq_read_lbl[index]
              end
  
              if(rq_write_lgn[index] == -1 and rq_read_lgn[index] != -1):
                  rq_write_lgn[index] = rq_read_lgn[index]
                  rq_write_lgn_previous[index] = rq_read_lgn[index]
              elif(rq_write_lgn_previous[index] == -1 and rq_read_lgn[index] != -1):
                  rq_write_lgn_previous[index] = rq_read_lgn[index]
              end
  
              if(rq_write_msc[index] == -1 and rq_read_msc[index] != -1):
                  rq_write_msc[index] = rq_read_msc[index]
                  rq_write_msc_previous[index] = rq_read_msc[index]
              elif(rq_write_msc_previous[index] == -1 and rq_read_msc[index] != -1):
                  rq_write_msc_previous[index] = rq_read_msc[index]
              end
  
              if(rq_write_mod[index] == -1 and rq_read_mod[index] != -1):
                  rq_write_mod[index] = rq_read_mod[index]
                  rq_write_mod_previous[index] = rq_read_mod[index]
              elif(rq_write_mod_previous[index] == -1 and rq_read_mod[index] != -1):
                  rq_write_mod_previous[index] = rq_read_mod[index]
              end
  
          end
          index = index + 1
      end
  end
  
  def rq_comm_set_var(var, value=0, gripper_socket="1"):
      socket_set_var(var, value, gripper_socket)
      return socket_read_byte_list(3, gripper_socket)
  end
  
  def rq_comm_set_pos_spe_for(pos=0, speed=0, force=0, gripper_socket="1"):
      socket_send_string("SET POS", gripper_socket)
      socket_send_byte(32, gripper_socket)
      socket_send_string(pos, gripper_socket)
      socket_send_byte(32, gripper_socket)
      socket_send_string("SPE", gripper_socket)
      socket_send_byte(32, gripper_socket)
      socket_send_string(speed, gripper_socket)
      socket_send_byte(32, gripper_socket)
      socket_send_string("FOR", gripper_socket)
      socket_send_byte(32, gripper_socket)
      socket_send_string(force, gripper_socket)
      socket_send_byte(10, gripper_socket)
      return socket_read_byte_list(3, gripper_socket)
  end
  
  def rq_comm_read_variables():
      index = 0
  
      while(index <= 3):
          socket = rq_index_to_socket(index)
  
          if(gripper_connected[index]):
  
              if(rq_read_act_req[index]):
                  rq_read_act[index] = socket_get_var("ACT", socket)
                  rq_read_act_req[index] = False
              end
  
              if(rq_read_gto_req[index]):
                  rq_read_gto[index] = socket_get_var("GTO", socket)
                  rq_read_gto_req[index] = False
              end
  
              if(rq_read_pre_req[index]):
                  rq_read_pre[index] = socket_get_var("PRE", socket)
                  rq_read_pre_req[index] = False
              end
  
              if(rq_read_pos_req[index]):
                  rq_read_pos[index] = socket_get_var("POS", socket)
                  rq_read_pos_req[index] = False
              end
  
              if(rq_read_spe_req[index]):
                  rq_read_spe[index] = socket_get_var("SPE", socket)
                  rq_read_spe_req[index] = False
              end
  
              if(rq_read_for_req[index]):
                  rq_read_for[index] = socket_get_var("FOR", socket)
                  rq_read_for_req[index] = False
              end
  
              if(rq_read_obj_req[index]):
                  rq_read_obj[index] = socket_get_var("OBJ", socket)
                  rq_read_obj_req[index] = False
              end
  
              if(rq_read_sta_req[index]):
                  rq_read_sta[index] = socket_get_var("STA", socket)
                  rq_read_sta_req[index] = False
              end
  
              if(rq_read_mod_req[index]):
                  rq_read_mod[index] = socket_get_var("MOD", socket)
                  rq_read_mod_req[index] = False
              end
  
              if(rq_read_flt_req[index]):
                  rq_read_flt[index] = socket_get_var("FLT",socket)
                  rq_read_flt_req[index] = False
              end
  
              if(rq_read_lbp_req[index]):
                  rq_read_lbp[index] = socket_get_var("LBP", socket)
                  rq_read_lbp_req[index] = False
              end
  
              if(rq_read_lrd_req[index]):
                  rq_read_lrd[index] = socket_get_var("LRD", socket)
                  rq_read_lrd_req[index] = False
              end
  
              if(rq_read_lbl_req[index]):
                  rq_read_lbl[index] = socket_get_var("LBL",socket)
                  rq_read_lbl_req[index] = False
              end
  
              if(rq_read_lgn_req[index]):
                  rq_read_lgn[index] = socket_get_var("LGN", socket)
                  rq_read_lgn_req[index] = False
              end
  
              if(rq_read_msc_req[index]):
                  rq_read_msc[index] = socket_get_var("MSC", socket)
                  rq_read_msc_req[index] = False
              end
  
              if(rq_read_cou_req[index]):
                  rq_read_cou[index] = socket_get_var("COU", socket)
                  rq_read_cou_req[index] = False
              end
  
              if(rq_read_ncy_req[index]):
                  rq_read_ncy[index] = socket_get_var("NCY", socket)
                  rq_read_ncy_req[index] = False
              end
  
              if(rq_read_dst_req[index]):
                  rq_read_dst[index] = socket_get_var("DST", socket)
                  rq_read_dst_req[index] = False
              end
  
              if(rq_read_pco_req[index]):
                  rq_read_pco[index] = socket_get_var("PCO", socket)
                  rq_read_pco_req[index] = False
              end
          end
          index = index + 1
      end
  end
  
  def rq_comm_read_constants():
  
      if(gripper_connected[0]):
          while(rq_read_snu_1 == rq_string_initial_value):
              socket_send_string("GET SNU", "1")
              rq_read_snu_1 = socket_read_string("1")
          end
  
          while(rq_read_fwv_1 == rq_string_initial_value):
              socket_send_string("GET FWV", "1")
              rq_read_fwv_1 = socket_read_string("1")
          end
  
          while(rq_read_ver_1 == rq_string_initial_value):
              socket_send_string("GET VER", "1")
              rq_read_ver_1 = socket_read_string("1")
          end
      end
  
      if(gripper_connected[1]):
          while(rq_read_snu_2 == rq_string_initial_value):
              socket_send_string("GET SNU", "2")
              rq_read_snu_2 = socket_read_string("2")
          end
  
          while(rq_read_fwv_2 == rq_string_initial_value):
              socket_send_string("GET FWV", "2")
              rq_read_fwv_2 = socket_read_string("2")
          end
  
          while(rq_read_ver_2 == rq_string_initial_value):
              socket_send_string("GET VER", "2")
              rq_read_ver_2 = socket_read_string("2")
          end
      end
  
      if(gripper_connected[2]):
          while(rq_read_snu_3 == rq_string_initial_value):
              socket_send_string("GET SNU", "3")
              rq_read_snu_3 = socket_read_string("3")
          end
  
          while(rq_read_fwv_3 == rq_string_initial_value):
              socket_send_string("GET FWV", "3")
              rq_read_fwv_3 = socket_read_string("3")
          end
  
          while(rq_read_ver_3 == rq_string_initial_value):
              socket_send_string("GET VER", "3")
              rq_read_ver_3 = socket_read_string("3")
          end
      end
  
      if(gripper_connected[3]):
          while(rq_read_snu_4 == rq_string_initial_value):
              socket_send_string("GET SNU", "4")
              rq_read_snu_4 = socket_read_string("4")
          end
  
          while(rq_read_fwv_4 == rq_string_initial_value):
              socket_send_string("GET FWV", "4")
              rq_read_fwv_4 = socket_read_string("4")
          end
  
          while(rq_read_ver_4 == rq_string_initial_value):
              socket_send_string("GET VER", "4")
              rq_read_ver_4 = socket_read_string("4")
          end
      end
  end
  
  def rq_comm_write_variables():
      index = 0
  
      while(index <= 3):
          socket = rq_index_to_socket(index)
  
          if(gripper_connected[index]):
  
              if(rq_write_act_request[index]):
                  if(is_ack(rq_comm_set_var("ACT", rq_write_act[index], socket))):
                      rq_write_act_previous[index] = rq_write_act[index]
                      rq_write_act_request[index] = False
                  end
              end
  
              if(rq_write_gto_request[index]):
                  if(is_ack(rq_comm_set_var("GTO", rq_write_gto[index], socket))):
                      rq_write_gto_previous[index] = rq_write_gto[index]
                      rq_write_gto_request[index] = False
                  end
              end
  
              if(rq_write_atr_request[index]):
                  if(is_ack(rq_comm_set_var("ATR", rq_write_atr[index], socket))):
                      rq_write_atr_previous[index] = rq_write_atr[index]
                      rq_write_atr_request[index] = False
                  end
              end
  
              if(rq_write_ard_request[index]):
                  if(is_ack(rq_comm_set_var("ARD", rq_write_ard[index], socket))):
                      rq_write_ard_previous[index] = rq_write_ard[index]
                      rq_write_ard_request[index] = False
                  end
              end
  
              if(rq_write_pos_request[index]):
                  if(is_ack(rq_comm_set_pos_spe_for(rq_write_pos[index], rq_write_spe[index], rq_write_for[index], socket))):
                      rq_write_pos_previous[index] = rq_write_pos[index]
                      rq_write_spe_previous[index] = rq_write_spe[index]
                      rq_write_for_previous[index] = rq_write_for[index]
                      rq_write_pos_request[index] = False
                  end
              end
  
              if(rq_write_lbp_request[index]):
                  if(is_ack(rq_comm_set_var("LBP", rq_write_lbp[index], socket))):
                      rq_write_lbp_previous[index] = rq_write_lbp[index]
                      rq_write_lbp_request[index] = False
                  end
              end
  
              if(rq_write_lrd_request[index]):
                  if(is_ack(rq_comm_set_var("LRD", rq_write_lrd[index], socket))):
                      rq_write_lrd_previous[index] = rq_write_lrd[index]
                      rq_write_lrd_request[index] = False
                  end
              end
  
              if(rq_write_lbl_request[index]):
                  if(is_ack(rq_comm_set_var("LBL", rq_write_lbl[index], socket))):
                      rq_write_lbl_previous[index] = rq_write_lbl[index]
                      rq_write_lbl_request[index] = False
                  end
              end
  
              if(rq_write_lgn_request[index]):
                  if(is_ack(rq_comm_set_var("LGN", rq_write_lgn[index], socket))):
                      rq_write_lgn_previous[index] = rq_write_lgn[index]
                      rq_write_lgn_request[index] = False
                  end
              end
  
              if(rq_write_msc_request[index]):
                  if(is_ack(rq_comm_set_var("MSC", rq_write_msc[index], socket))):
                      rq_write_msc_previous[index] = rq_write_msc[index]
                      rq_write_msc_request[index] = False
                  end
              end
  
              if(rq_write_mod_request[index]):
                  if(is_ack(rq_comm_set_var("MOD", rq_write_mod[index], socket))):
                      rq_write_mod_previous[index] = rq_write_mod[index]
                      rq_write_mod_request[index] = False
                  end
              end
          end
          index = index + 1
      end
  end
  
  def rq_activate(gripper_socket="1"):
      if (not rq_is_gripper_activated(gripper_socket)):
          rq_reset(gripper_socket)
  
          while(not rq_get_var("ACT", 1, gripper_socket) == 0 or not rq_get_var("STA", 1, gripper_socket) == 0):
              rq_reset(gripper_socket)
              sync()
          end
  
          rq_set_var("ACT",1, gripper_socket)
      end
  end
  
  def rq_activate_and_wait(gripper_socket="1"):
      if (not rq_is_gripper_activated(gripper_socket)):
          rq_activate(gripper_socket)
          sleep(1.0)
  
          while(not rq_get_var("ACT", 1, gripper_socket) == 1 or not rq_get_var("STA", 1, gripper_socket) == 3):
              sleep(0.1)
          end
  
          sleep(0.5)
      end
  end
  
  def rq_activate_all_grippers(reset=False):
      if(gripper_connected[0]):
          rq_reset_and_activate("1", reset)
      end
  
      if(gripper_connected[1]):
          rq_reset_and_activate("2", reset)
      end
  
      if(gripper_connected[2]):
          rq_reset_and_activate("3", reset)
      end
  
      if(gripper_connected[3]):
          rq_reset_and_activate("4", reset)
      end
  end
  
  def rq_reset_and_activate(gripper_socket="1", reset=False):
      if(reset):
          rq_reset(gripper_socket)
          rq_activate_and_wait(gripper_socket)
      elif(not rq_is_gripper_activated(gripper_socket)):
          rq_activate_and_wait(gripper_socket)
      end
  end
  
  def rq_reset(gripper_socket="1"):
      rq_set_var("ACT", 0, gripper_socket)
      rq_set_var("ATR", 0, gripper_socket)
  
      while(not rq_get_var("ACT", 1, gripper_socket) == 0 or not rq_get_var("STA", 1, gripper_socket) == 0):
          rq_set_var("ACT", 0, gripper_socket)
          rq_set_var("ATR", 0, gripper_socket)
          sync()
      end
  
      sleep(0.5)
  end
  
  def rq_auto_release_open_and_wait(gripper_socket="1"):
      rq_set_var("ATR",0, gripper_socket)
      rq_set_var("ARD",0, gripper_socket)
      rq_set_var("ACT",1, gripper_socket)
      sleep(0.1)
      rq_set_var("ATR",1, gripper_socket)
  
      rq_wait_autorelease_completed(gripper_socket)
  end
  
  def rq_auto_release_close_and_wait(gripper_socket="1"):
      rq_set_var("ATR",0, gripper_socket)
      rq_set_var("ARD",1, gripper_socket)
      rq_set_var("ACT",1, gripper_socket)
      sleep(0.1)
      rq_set_var("ATR",1, gripper_socket)
  
      rq_wait_autorelease_completed(gripper_socket)
  end
  
  def rq_wait_autorelease_completed(gripper_socket="1"):
      remainingRetries = 20
      gFLT = rq_get_var("FLT", 2, gripper_socket)
  
      while(not is_FLT_autorelease_in_progress(gFLT) and remainingRetries > 0):
          remainingRetries = remainingRetries - 1
          gFLT = rq_get_var("FLT", 2, gripper_socket)
          sleep(0.1)
      end
  
      remainingRetries = 100
      gFLT = rq_get_var("FLT", 2, gripper_socket)
  
      while(not is_FLT_autorelease_completed(gFLT) and remainingRetries > 0):
          remainingRetries = remainingRetries - 1
          gFLT = rq_get_var("FLT", 2, gripper_socket)
          sleep(0.1)
      end
  end
  
  def rq_set_force(force, gripper_socket="1"):
      force = floor(scale(force, [0, 255], [0.0, 255.0]))
      rq_set_var("FOR", force, gripper_socket)
  end
  
  def rq_set_speed(speed, gripper_socket="1"):
      speed = floor(scale(speed, [0, 255], [0.0, 255.0]))
      rq_set_var("SPE", speed, gripper_socket)
  end
  
  def rq_open(gripper_socket="1"):
      rq_move(0, gripper_socket)
  end
  
  def rq_close(gripper_socket="1"):
      rq_move(255, gripper_socket)
  end
  
  def rq_open_and_wait(gripper_socket="1"):
      rq_move_and_wait(0, gripper_socket)
  end
  
  def rq_close_and_wait(gripper_socket="1"):
      rq_move_and_wait(255, gripper_socket)
  end
  
  def rq_move(pos, gripper_socket="1"):
      rq_set_pos(pos, gripper_socket)
      rq_go_to(gripper_socket)
  end
  
  def rq_move_and_wait(pos, gripper_socket="1"):
      rq_move(pos, gripper_socket)
  
      while (not rq_is_motion_complete(gripper_socket)):
          # wait for motion completed
          sync()
      end
  end
  
  def rq_wait_for_pos_request(pos, gripper_socket="1"):
      gPRE = rq_get_var("PRE", 3, gripper_socket)
  
      while (gPRE != pos):
          rq_set_var("POS", pos, gripper_socket)
          sync()
          gPRE = rq_get_var("PRE", 3, gripper_socket)
      end
  end
  
  def rq_wait_pos_spe_for_request(pos, speed, force, gripper_socket="1"):
      gPRE = rq_get_var("PRE", 3, gripper_socket)
  
      while (gPRE != pos):
          rq_set_pos_spe_for_var(pos, speed, force, gripper_socket)
          sync()
          gPRE = rq_get_var("PRE", 3, gripper_socket)
      end
  end
  
  def rq_wait_for_pos(pos, gripper_socket="1"):
      rq_wait_for_pos_request(pos, gripper_socket)
  
      # Wait for the gripper motion to complete
      while (not rq_is_motion_complete(gripper_socket)):
          # wait for motion completed
          sync()
          rq_set_var("GTO", 1, gripper_socket)
      end
  end
  
  def rq_wait(gripper_socket="1"):
      # Wait for the gripper motion to complete
      while (not rq_is_motion_complete(gripper_socket)):
          sync()
          # The following patch is for Robotiq's Camera issue when communication is lost, but not the activation
          # the communication driver reset the GTO bit
          rq_set_var("GTO", 1, gripper_socket)
      end
  end
  
  def rq_wait_for_object_detected(gripper_socket="1"):
      # Wait the object detection
      while (not rq_is_object_detected(gripper_socket)):
          sync()
      end
  end
  
  # set the position
  def rq_set_pos(pos, gripper_socket="1"):
      pos = floor(scale(pos, [0, 255], [0.0, 255.0]))
      rq_set_var("POS", pos, gripper_socket)
      rq_wait_for_pos_request(pos, gripper_socket)
  end
  
  def rq_set_pos_spd_for(pos, speed, force, gripper_socket="1"):
      pos = floor(scale(pos, [0, 255], [0.0, 255.0]))
      speed = floor(scale(speed, [0, 255], [0.0, 255.0]))
      force = floor(scale(force, [0, 255], [0.0, 255.0]))
  
      rq_set_pos_spe_for_var(pos, speed, force, gripper_socket)
  end
  
  def rq_set_gripper_max_current_mA(current_mA, gripper_socket="1"):
      current = floor(current_mA / 10)
  
      rq_set_var("MSC", current, gripper_socket)
      current_read = rq_get_var("MSC", 1, gripper_socket)
  
      while(current_read != current):
          rq_set_var("MSC", current, gripper_socket)
          current_read = rq_get_var("MSC", 1, gripper_socket)
      end
  end
  
  def rq_set_gripper_mode(mode, gripper_socket="1"):
      rq_set_var("MOD", mode, gripper_socket)
  end
  
  def rq_set_gripper_max_cur(current_mA, gripper_socket="1"):
      rq_set_gripper_max_current_mA(current_mA, gripper_socket)
  end
  
  def rq_get_gripper_max_current_mA(gripper_socket="1"):
      current = rq_get_var("MSC", 1, gripper_socket)
  
      if(current == -1):
          current_mA = current
      else:
          current_mA = current * 10
      end
  
      return current_mA
  end
  
  def rq_get_gripper_max_cur(gripper_socket="1"):
      return rq_get_gripper_max_current_mA(gripper_socket)
  end
  
  def rq_set_max_current_for_all_grippers():
      current_mA = rq_get_max_current_mA()
  
      if(gripper_connected[0]):
          rq_set_gripper_max_current_mA(current_mA, "1")
      end
  
      if(gripper_connected[1]):
          rq_set_gripper_max_current_mA(current_mA, "2")
      end
  
      if(gripper_connected[2]):
          rq_set_gripper_max_current_mA(current_mA, "3")
      end
  
      if(gripper_connected[3]):
          rq_set_gripper_max_current_mA(current_mA, "4")
      end
  end
  
  def rq_get_max_current_mA():
      max_current_mA = 0
  
      if(rq_current_limit_enabled):
          nb_connected_grippers = rq_get_nb_connected_grippers()
  
          if(nb_connected_grippers == 1):
              max_current_mA = 600
          elif(nb_connected_grippers > 1):
              max_current_mA = 450
          end
      else:
  
      end
  
      return max_current_mA
  end
  
  def rq_get_nb_connected_grippers():
      nb_connected_grippers = 0
  
      if(gripper_connected[0]):
          nb_connected_grippers = nb_connected_grippers + 1
      end
  
      if(gripper_connected[1]):
          nb_connected_grippers = nb_connected_grippers + 1
      end
  
      if(gripper_connected[2]):
          nb_connected_grippers = nb_connected_grippers + 1
      end
  
      if(gripper_connected[3]):
          nb_connected_grippers = nb_connected_grippers + 1
      end
  
      return nb_connected_grippers
  end
  
  def rq_list_of_bytes_to_value(list_of_bytes):
      value = -1
  
      # response list length
      if (list_of_bytes[0] == 1):
          value = list_of_bytes[1] - 48
      elif (list_of_bytes[0] == 2):
          value = (list_of_bytes[1] - 48) * 10 + (list_of_bytes[2] - 48)
      elif (list_of_bytes[0] == 3):
          value = (list_of_bytes[1] - 48) * 100 + (list_of_bytes[2] - 48) * 10 + (list_of_bytes[3] - 48)
      end
  
      return value
  end
  
  def rq_is_motion_complete(gripper_socket="1"):
      gOBJ = rq_get_var("OBJ", 1, gripper_socket)
      sync()
      return is_OBJ_gripper_at_position(gOBJ) or is_OBJ_object_detected(gOBJ)
  end
  
  def rq_is_gripper_activated(gripper_socket="1"):
      gSTA = rq_get_var("STA", 1, gripper_socket)
      sync()
      return is_STA_gripper_activated(gSTA)
  end
  
  def rq_is_object_detected(gripper_socket="1"):
      gOBJ = rq_get_var("OBJ", 1, gripper_socket)
      sync()
      return is_OBJ_object_detected(gOBJ)
  end
  
  def rq_current_pos(gripper_socket="1"):
      gPOS = rq_get_var("POS", 1, gripper_socket)
      sync()
      return gPOS
  end
  
  def rq_motor_current(gripper_socket="1"):
      rq_current = rq_get_var("COU", 1, gripper_socket)
      sync()
      return rq_current * 10
  end
  
  def rq_print_connected_grippers():
      if(gripper_connected[0]):
          textmsg("Gripper 1 : ", "connected and socket open.")
      end
  
      if (gripper_connected[1]):
          textmsg("Gripper 2 : ", "connected and socket open.")
      end
  
      if (gripper_connected[2]):
          textmsg("Gripper 3 : ", "connected and socket open.")
      end
  
      if (gripper_connected[3]):
          textmsg("Gripper 4 : ", "connected and socket open.")
      end
  end
  
  def rq_print_gripper_fault_code(gripper_socket="1"):
      gFLT = rq_get_var("FLT", 2, gripper_socket)
  
      if(is_FLT_no_fault(gFLT)):
          textmsg("Gripper Fault : ", "No Fault (0x00)")
      elif (is_FLT_action_delayed(gFLT)):
          textmsg("Gripper Fault : ", "Priority Fault: Action delayed, initialization must be completed prior to action (0x05)")
      elif (is_FLT_not_activated(gFLT)):
          textmsg("Gripper Fault : ", "Priority Fault: The activation must be set prior to action (0x07)")
      elif (is_FLT_autorelease_in_progress(gFLT)):
          textmsg("Gripper Fault : ", "Minor Fault: Automatic release in progress (0x0B)")
      elif (is_FLT_overcurrent(gFLT)):
          textmsg("Gripper Fault : ", "Minor Fault: Overcurrent protection triggered (0x0E)")
      elif (is_FLT_autorelease_completed(gFLT)):
          textmsg("Gripper Fault : ", "Major Fault: Automatic release completed (0x0F)")
      else:
          textmsg("Gripper Fault : ", "Unknown Fault")
      end
  end
  
  def rq_print_gripper_num_cycles(gripper_socket="1"):
      num_cycles = rq_get_var("NCY", 1, gripper_socket)
  
      if(num_cycles == -1):
          textmsg("Gripper Cycle Number : ", "Number of cycles is unreachable.")
      else:
          textmsg("Gripper Cycle Number : ", num_cycles)
      end
  end
  
  def rq_print_gripper_driver_state(gripper_socket="1"):
      driver_state = rq_get_var("DST", 1, gripper_socket)
  
      if(driver_state == 0):
          textmsg("Gripper Driver State : ", "RQ_STATE_INIT")
      elif(driver_state == 1):
          textmsg("Gripper Driver State : ", "RQ_STATE_LISTEN")
      elif(driver_state == 2):
          textmsg("Gripper Driver State : ", "RQ_STATE_READ_INFO")
      elif(driver_state == 3):
          textmsg("Gripper Driver State : ", "RQ_STATE_ACTIVATION")
      else:
          textmsg("Gripper Driver State : ", "RQ_STATE_RUN")
      end
  end
  
  def rq_print_gripper_serial_number(gripper_socket="1"):
      serial_number = rq_get_var_string("SNU", 1, gripper_socket)
      textmsg("Gripper Serial Number : ", serial_number)
  end
  
  def rq_print_gripper_firmware_version(gripper_socket="1"):
      firmware_version = rq_get_var_string("FWV", 1, gripper_socket)
      textmsg("Gripper Firmware Version : ", firmware_version)
  end
  
  def rq_print_gripper_driver_version(gripper_socket="1"):
      driver_version = rq_get_var_string("VER", 1, gripper_socket)
      textmsg("Gripper Driver Version : ", driver_version)
  end
  
  def rq_print_gripper_connection_state(gripper_socket="1"):
      connection_state = rq_get_var("PCO", 1, gripper_socket)
  
      if (connection_state == 0):
          textmsg("Gripper Connection State : ", "No connection problem detected")
      else:
          textmsg("Gripper Connection State : ", "Connection problem detected")
      end
  end
  
  # Returns True if list_of_bytes is [3, 'a', 'c', 'k']
  def is_ack(list_of_bytes):
  
      # list length is not 3
      if (list_of_bytes[0] != 3):
          return False
      end
  
      # first byte not is 'a'?
      if (list_of_bytes[1] != 97):
          return False
      end
  
      # first byte not is 'c'?
      if (list_of_bytes[2] != 99):
          return False
      end
  
      # first byte not is 'k'?
      if (list_of_bytes[3] != 107):
          return False
      end
  
      return True
  end
  
  # Returns True if list_of_bytes is not [3, 'a', 'c', 'k']
  def is_not_ack(list_of_bytes):
      if (is_ack(list_of_bytes)):
          return False
      else:
          return True
      end
  end
  
  def is_STA_gripper_activated (gSTA):
      if (gSTA == 3):
          return True
      end
  
      return False
  end
  
  def is_OBJ_object_detected (gOBJ):
      if (gOBJ == 1 or gOBJ == 2):
          return True
      end
  
      return False
  end
  
  def is_OBJ_gripper_at_position (gOBJ):
      if (gOBJ == 3):
          return True
      end
  
      return False
  end
  
  def is_not_OBJ_gripper_at_position (gOBJ):
      if (is_OBJ_gripper_at_position(gOBJ)):
          return False
      else:
          return True
      end
  end
  
  #### GTO Section ####
  def rq_stop(gripper_socket="1"):
      rq_set_var("GTO", 0, gripper_socket)
  end
  
  def rq_set_GTO_and_wait(value, gripper_socket="1"):
      rq_set_var("GTO" ,value, gripper_socket)
      while(not is_GTO(value, rq_get_var("GTO", 1, gripper_socket))):
        sync()
      end
  end
  
  def rq_go_to(gripper_socket="1"):
      rq_set_var("GTO", 1, gripper_socket)
  end
  
  
  def is_GTO(goto_value, rGTO):
      return rGTO == goto_value
  end
  #### GTO Section ####
  
  def is_FLT_no_fault(gFLT):
      return gFLT == 0
  end
  
  def is_FLT_warning(gFLT):
      return gFLT >= 1 and gFLT <= 7
  end
  
  def is_FLT_faulted(gFLT):
      return gFLT >= 8
  end
  
  def is_FLT_action_delayed(gFLT):
      return gFLT == 5
  end
  
  def is_FLT_not_activated(gFLT):
      return gFLT == 7
  end
  
  def is_FLT_autorelease_in_progress(gFLT):
      return gFLT == 11
  end
  
  def is_FLT_overcurrent(gFLT):
      return gFLT == 14
  end
  
  def is_FLT_autorelease_completed(gFLT):
      return gFLT == 15
  end
  
  def rq_set_var(var_name, var_value, gripper_socket="1"):
      index = rq_socket_to_index(gripper_socket)
  
      enter_critical
  
      if (var_name == "ACT"):
          rq_write_act[index] = var_value
          rq_write_act_request[index] = True
      elif (var_name == "GTO"):
          rq_write_gto[index] = var_value
          rq_write_gto_request[index] = True
      elif (var_name == "ATR"):
          rq_write_atr[index] = var_value
          rq_write_atr_request[index] = True
      elif (var_name == "ARD"):
          rq_write_ard[index] = var_value
          rq_write_ard_request[index] = True
      elif (var_name == "FOR"):
          rq_write_for[index] = var_value
          rq_write_pos_request[index] = True
      elif (var_name == "SPE"):
          rq_write_spe[index] = var_value
          rq_write_pos_request[index] = True
      elif (var_name == "POS"):
          rq_write_pos[index] = var_value
          rq_write_pos_request[index] = True
      elif (var_name == "LBP"):
          rq_write_lbp[index] = var_value
          rq_write_lbp_request[index] = True
      elif (var_name == "LRD"):
          rq_write_lrd[index] = var_value
          rq_write_lrd_request[index] = True
      elif (var_name == "LBL"):
          rq_write_lbl[index] = var_value
          rq_write_lbl_request[index] = True
      elif (var_name == "LGN"):
          rq_write_lgn[index] = var_value
          rq_write_lgn_request[index] = True
      elif (var_name == "MSC"):
          rq_write_msc[index] = var_value
          rq_write_msc_request[index] = True
      elif (var_name == "MOD"):
          rq_write_mod[index] = var_value
          rq_write_mod_request[index] = True
      end
  
      exit_critical
  
      if (var_name == "ACT"):
          while(rq_write_act_request[index]):
              sync()
          end
      elif (var_name == "GTO"):
          while(rq_write_gto_request[index]):
              sync()
          end
      elif (var_name == "ATR"):
          while(rq_write_atr_request[index]):
              sync()
          end
      elif (var_name == "ARD"):
          while(rq_write_ard_request[index]):
              sync()
          end
      elif (var_name == "FOR"):
          while(rq_write_pos_request[index]):
              sync()
          end
      elif (var_name == "SPE"):
          while(rq_write_pos_request[index]):
              sync()
          end
      elif (var_name == "POS"):
          while(rq_write_pos_request[index]):
              sync()
          end
      elif (var_name == "LBP"):
          while(rq_write_lbp_request[index]):
              sync()
          end
      elif (var_name == "LRD"):
          while(rq_write_lrd_request[index]):
              sync()
          end
      elif (var_name == "LBL"):
          while(rq_write_lbl_request[index]):
              sync()
          end
      elif (var_name == "LGN"):
          while(rq_write_lgn_request[index]):
              sync()
          end
      elif (var_name == "MSC"):
          while(rq_write_msc_request[index]):
              sync()
          end
      elif (var_name == "MOD"):
          while(rq_write_mod_request[index]):
              sync()
          end
      end
  end
  
  def rq_set_pos_spe_for_var(pos, speed, force, gripper_socket="1"):
      index = rq_socket_to_index(gripper_socket)
  
      enter_critical
  
      rq_write_for[index] = force
      rq_write_spe[index] = speed
      rq_write_pos[index] = pos
      rq_write_pos_request[index] = True
  
      exit_critical
  end
  
  def rq_get_var(var_name, nbr_bytes, gripper_socket="1"):
      index = rq_socket_to_index(gripper_socket)
      var_value = -1
  
      if (var_name == "ACT"):
          enter_critical
          rq_read_act_req[index] = True
          exit_critical
          while(rq_read_act_req[index]):
              sync()
          end
          var_value = rq_read_act[index]
  
      elif (var_name == "GTO"):
          enter_critical
          rq_read_gto_req[index] = True
          exit_critical
          while(rq_read_gto_req[index]):
              sync()
          end
          var_value = rq_read_gto[index]
  
      elif (var_name == "FOR"):
          enter_critical
          rq_read_for_req[index] = True
          exit_critical
          while(rq_read_for_req[index]):
              sync()
          end
          var_value = rq_read_for[index]
  
      elif (var_name == "SPE"):
          enter_critical
          rq_read_spe_req[index] = True
          exit_critical
          while(rq_read_spe_req[index]):
              sync()
          end
          var_value = rq_read_spe[index]
  
      elif (var_name == "OBJ"):
          enter_critical
          rq_read_obj_req[index] = True
          exit_critical
          while(rq_read_obj_req[index]):
              sync()
          end
          var_value = rq_read_obj[index]
  
      elif (var_name == "STA"):
          enter_critical
          rq_read_sta_req[index] = True
          exit_critical
          while(rq_read_sta_req[index]):
              sync()
          end
          var_value = rq_read_sta[index]
  
      elif (var_name == "FLT"):
          enter_critical
          rq_read_flt_req[index] = True
          exit_critical
          while(rq_read_flt_req[index]):
              sync()
          end
          var_value = rq_read_flt[index]
  
      elif (var_name == "POS"):
          enter_critical
          rq_read_pos_req[index] = True
          exit_critical
          while(rq_read_pos_req[index]):
              sync()
          end
          var_value = rq_read_pos[index]
  
      elif (var_name == "PRE"):
          enter_critical
          rq_read_pre_req[index] = True
          exit_critical
          while(rq_read_pre_req[index]):
              sync()
          end
          var_value = rq_read_pre[index]
  
      elif (var_name == "LBP"):
          enter_critical
          rq_read_lbp_req[index] = True
          exit_critical
          while(rq_read_lbp_req[index]):
              sync()
          end
          var_value = rq_read_lbp[index]
  
      elif (var_name == "LRD"):
          enter_critical
          rq_read_lrd_req[index] = True
          exit_critical
          while(rq_read_lrd_req[index]):
              sync()
          end
          var_value = rq_read_lrd[index]
  
      elif (var_name == "LBL"):
          enter_critical
          rq_read_lbl_req[index] = True
          exit_critical
          while(rq_read_lbl_req[index]):
              sync()
          end
          var_value = rq_read_lbl[index]
  
      elif (var_name == "LGN"):
          enter_critical
          rq_read_lgn_req[index] = True
          exit_critical
          while(rq_read_lgn_req[index]):
              sync()
          end
          var_value = rq_read_lgn[index]
  
      elif (var_name == "MSC"):
          enter_critical
          rq_read_msc_req[index] = True
          exit_critical
          while(rq_read_msc_req[index]):
              sync()
          end
          var_value = rq_read_msc[index]
  
      elif (var_name == "MOD"):
          enter_critical
          rq_read_mod_req[index] = True
          exit_critical
          while(rq_read_mod_req[index]):
              sync()
          end
          var_value = rq_read_mod[index]
  
      elif (var_name == "NCY"):
          enter_critical
          rq_read_ncy_req[index] = True
          exit_critical
          while(rq_read_ncy_req[index]):
              sync()
          end
          var_value = rq_read_ncy[index]
  
      elif (var_name == "PCO"):
          enter_critical
          rq_read_pco_req[index] = True
          exit_critical
          while(rq_read_pco_req[index]):
              sync()
          end
          var_value = rq_read_pco[index]
  
      elif (var_name == "DST"):
          enter_critical
          rq_read_dst_req[index] = True
          exit_critical
          while(rq_read_dst_req[index]):
              sync()
          end
          var_value = rq_read_dst[index]
  
      end
  
      return var_value
  end
  
  def rq_get_var_string(var_name, nbr_bytes, gripper_socket="1"):
      index = rq_socket_to_index(gripper_socket)
      var_value = ""
  
      enter_critical
  
      if (var_name == "SNU"):
          if(gripper_socket == "1"):
              var_value = rq_read_snu_1
          elif(gripper_socket == "2"):
              var_value = rq_read_snu_2
          elif(gripper_socket == "3"):
              var_value = rq_read_snu_3
          elif(gripper_socket == "4"):
              var_value = rq_read_snu_4
          end
      elif (var_name == "FWV"):
          if(gripper_socket == "1"):
              var_value = rq_read_fwv_1
          elif(gripper_socket == "2"):
              var_value = rq_read_fwv_2
          elif(gripper_socket == "3"):
              var_value = rq_read_fwv_3
          elif(gripper_socket == "4"):
              var_value = rq_read_fwv_4
          end
      elif (var_name == "VER"):
          if(gripper_socket == "1"):
              var_value = rq_read_ver_1
          elif(gripper_socket == "2"):
              var_value = rq_read_ver_2
          elif(gripper_socket == "3"):
              var_value = rq_read_ver_3
          elif(gripper_socket == "4"):
              var_value = rq_read_ver_4
          end
      end
  
      exit_critical
  
      return var_value
  end
  
  def rq_is_object_validated(gripper_selected, gripper_socket="1"):
      if(gripper_selected):
          if(rq_is_object_detected(gripper_socket)):
              return True
          else:
              return False
          end
      else:
          return True
      end
  end
  
  ############################################
  # normalized functions (maps 0-100 to 0-255)
  ############################################
  def rq_set_force_norm(force_norm, gripper_socket="1"):
      force_gripper = norm_to_gripper(force_norm)
      rq_set_force(force_gripper, gripper_socket)
  end
  
  def rq_set_speed_norm(speed_norm, gripper_socket="1"):
      speed_gripper = norm_to_gripper(speed_norm)
      rq_set_speed(speed_gripper, gripper_socket)
  end
  
  def rq_move_norm(pos_norm, gripper_socket="1"):
      pos_gripper = norm_to_gripper(pos_norm)
      rq_move(pos_gripper, gripper_socket)
  end
  
  def rq_move_and_wait_norm(pos_norm, gripper_socket="1"):
      pos_gripper = norm_to_gripper(pos_norm)
      rq_move_and_wait(pos_gripper, gripper_socket)
  end
  
  def rq_set_pos_norm(pos_norm, gripper_socket="1"):
      pos_gripper = norm_to_gripper(pos_norm)
      rq_set_pos(pos_gripper, gripper_socket)
  end
  
  def rq_current_pos_norm(gripper_socket="1"):
      pos_gripper = rq_current_pos(gripper_socket)
      pos_norm = gripper_to_norm(pos_gripper)
      return pos_norm
  end
  
  def gripper_to_norm(value_gripper):
      value_norm = (value_gripper / 255) * 100
      return floor(value_norm)
  end
  
  def norm_to_gripper(value_norm):
      value_gripper = (value_norm / 100) * 255
      return ceil(value_gripper)
  end
  
  def rq_get_position():
      return rq_current_pos_norm()
  end
  
  def rq_gripper_led_on(gripper_socket="1"):
      rq_set_var("LBP",0, gripper_socket)
  end
  
  def rq_gripper_led_off(gripper_socket="1"):
      rq_set_var("LBP",1, gripper_socket)
      rq_set_var("LRD",0, gripper_socket)
      rq_set_var("LBL",0, gripper_socket)
      rq_set_var("LGN",0, gripper_socket)
  end
  
  def rq_gripper_led_force_red(gripper_socket="1"):
      rq_set_var("LBP",1, gripper_socket)
      rq_set_var("LRD",1, gripper_socket)
      rq_set_var("LBL",0, gripper_socket)
      rq_set_var("LGN",0, gripper_socket)
  end
  
  def rq_gripper_led_force_blue(gripper_socket="1"):
      rq_set_var("LBP",1, gripper_socket)
      rq_set_var("LRD",0, gripper_socket)
      rq_set_var("LBL",1, gripper_socket)
      rq_set_var("LGN",0, gripper_socket)
  end
  
  def rq_gripper_led_force_green(gripper_socket="1"):
      rq_set_var("LBP",1, gripper_socket)
      rq_set_var("LRD",0, gripper_socket)
      rq_set_var("LBL",0, gripper_socket)
      rq_set_var("LGN",1, gripper_socket)
  end
  
  def rq_gripper_led_force_purple(gripper_socket="1"):
      rq_set_var("LBP",1, gripper_socket)
      rq_set_var("LRD",1, gripper_socket)
      rq_set_var("LBL",1, gripper_socket)
      rq_set_var("LGN",0, gripper_socket)
  end
  
  ############################################
  # mm/inches functions
  ############################################
  gripper_closed_norm = [100, 100, 100, 100]
  gripper_open_norm = [0, 0, 0, 0]
  gripper_closed_mm = [0, 0, 0, 0]
  gripper_open_mm = [50, 50, 50, 50]
  
  def rq_current_pos_mm(gripper_socket=1):
      pos_gripper = rq_current_pos(gripper_socket)
      pos_mm = gripper_to_mm(pos_gripper, gripper_socket)
      return round_value_2_dec(pos_mm)
  end
  
  def rq_current_pos_inches(gripper_socket=1):
      pos_gripper = rq_current_pos(gripper_socket)
      pos_mm = gripper_to_mm(pos_gripper, gripper_socket)
      pos_in = pos_mm / 25.4
      return round_value_2_dec(pos_in)
  end
  
  def rq_move_mm(pos_mm, gripper_socket=1):
      pos_gripper = mm_to_gripper(pos_mm, gripper_socket)
      rq_move(pos_gripper, gripper_socket)
  end
  
  def rq_move_and_wait_mm(pos_mm, gripper_socket=1):
      pos_gripper = mm_to_gripper(pos_mm, gripper_socket)
      rq_move_and_wait(pos_gripper, gripper_socket)
  end
  
  def rq_move_inches(pos_in, gripper_socket=1):
      pos_mm = pos_in * 25.4
      rq_move_mm(pos_mm, gripper_socket)
  end
  
  def rq_move_and_wait_inches(pos_in, gripper_socket=1):
      pos_mm = pos_in * 25.4
      rq_move_and_wait_mm(pos_mm, gripper_socket)
  end
  
  def get_closed_norm(gripper_socket):
      return gripper_closed_norm[gripper_socket - 1]
  end
  
  def get_open_norm(gripper_socket):
      return gripper_open_norm[gripper_socket - 1]
  end
  
  def get_closed_mm(gripper_socket):
      return gripper_closed_mm[gripper_socket - 1]
  end
  
  def get_open_mm(gripper_socket):
      return gripper_open_mm[gripper_socket - 1]
  end
  
  def set_closed_norm(closed_norm, gripper_socket):
      gripper_closed_norm[gripper_socket - 1] = closed_norm
  end
  
  def set_open_norm(open_norm, gripper_socket):
      gripper_open_norm[gripper_socket - 1] = open_norm
  end
  
  def set_closed_mm(closed_mm, gripper_socket):
      gripper_closed_mm[gripper_socket - 1] = closed_mm
  end
  
  def set_open_mm(open_mm, gripper_socket):
      gripper_open_mm[gripper_socket - 1] = open_mm
  end
  
  def gripper_to_mm(value_gripper, gripper_socket):
      closed_norm = get_closed_norm(gripper_socket)
      open_norm = get_open_norm(gripper_socket)
      closed_mm = get_closed_mm(gripper_socket)
      open_mm = get_open_mm(gripper_socket)
  
      value_norm = (value_gripper / 255) * 100
  
      slope = (closed_mm - open_mm) / (closed_norm - open_norm)
      value_mm = slope * (value_norm - closed_norm) + closed_mm
  
      if (value_mm > open_mm):
          value_mm_limited = open_mm
      elif (value_mm < closed_mm):
          value_mm_limited = closed_mm
      else:
          value_mm_limited = value_mm
      end
  
      return value_mm_limited
  end
  
  def mm_to_gripper(value_mm, gripper_socket):
      closed_norm = get_closed_norm(gripper_socket)
      open_norm = get_open_norm(gripper_socket)
      closed_mm = get_closed_mm(gripper_socket)
      open_mm = get_open_mm(gripper_socket)
  
      slope = (closed_norm - open_norm) / (closed_mm - open_mm)
      value_norm = (value_mm - closed_mm) * slope + closed_norm
  
      value_gripper = value_norm * 255 / 100
  
      if (value_gripper > 255):
          value_gripper_limited = 255
      elif (value_gripper < 0):
          value_gripper_limited = 0
      else:
          value_gripper_limited = round_value(value_gripper)
      end
  
      return value_gripper_limited
  end
  
  def round_value(value):
      value_mod = value % 1
  
      if(value_mod < 0.5):
          return floor(value)
      else:
          return ceil(value)
      end
  end
  
  def round_value_2_dec(value):
      value_x_100 = value * 100
      value_x_100_rounded = round_value(value_x_100)
      return value_x_100_rounded / 100
  end
  
  def clear_socket_buffer(gripper_socket="1", read_timeout = 0.1):
      rq_comm_clear_socket_buffer_enabled[rq_socket_to_index(gripper_socket)] = True
  end
  
  def rq_gripper_id_to_ascii(gripper_id):
      if(gripper_id == "1"):
          return 57
      elif(gripper_id == "2"):
          return 50
      elif(gripper_id == "3"):
          return 51
      elif(gripper_id == "4"):
          return 52
      end
  end
  
  def scale(value, rawRange, scaledRange):
      def computeSlope(inputRange, outputRange):
          outputRangeDelta = outputRange[1] - outputRange[0]
          inputRangeDelta = inputRange[1] - inputRange[0]
  
          if (inputRangeDelta == 0):
              return 0
          else:
              return outputRangeDelta / inputRangeDelta
          end
      end
  
      def computeIntercept(slope, inputRange, outputRange):
          return outputRange[0] - (slope * inputRange[0])
      end
  
      def clipScaledValue(outputScaledValue, outputRange):
          if (outputRange[0] < outputRange[1]):
              return clipWhenLowerLimitIsLessThanHigher(outputScaledValue, outputRange)
          else:
              return clipWhenLowerLimitIsGreaterThanHigherLimit(outputScaledValue, outputRange)
          end
      end
  
      def clipWhenLowerLimitIsGreaterThanHigherLimit(outputScaledValue, outputRange):
          if (outputScaledValue < outputRange[1]):
              return outputRange[1]
          elif (outputScaledValue > outputRange[0]):
              return outputRange[0]
          else:
              return outputScaledValue
          end
      end
  
      def clipWhenLowerLimitIsLessThanHigher(outputScaledValue, outputRange):
          if (outputScaledValue < outputRange[0]):
              return outputRange[0]
          elif (outputScaledValue > outputRange[1]):
              return outputRange[1]
          else:
              return outputScaledValue
          end
      end
  
      slope = computeSlope(rawRange, scaledRange)
      intercept = computeIntercept(slope, rawRange, scaledRange)
      scaledValue = slope * value + intercept
      return clipScaledValue(scaledValue, scaledRange)
  end
  
  def limit(value, range):
      return scale(value, range, range)
  end
  
  
  rq_init_comm_if_connected(9, "1")
  rq_init_comm_if_connected(2, "2")
  rq_init_comm_if_connected(3, "3")
  rq_init_comm_if_connected(4, "4")
  rq_print_connected_grippers()
  rq_gripper_communication_thread = run rq_gripper_communication()
  set_closed_norm(100.0, 1)
  set_open_norm(0.0, 1)
  set_closed_mm(0.0, 1)
  set_open_mm(50.0, 1)
  set_closed_norm(98.0392156862745, 2)
  set_open_norm(1.1764705882352942, 2)
  set_closed_mm(0.0, 2)
  set_open_mm(50.0, 2)
  set_closed_norm(100.0, 3)
  set_open_norm(0.0, 3)
  set_closed_mm(0.0, 3)
  set_open_mm(50.0, 3)
  set_closed_norm(100.0, 4)
  set_open_norm(0.0, 4)
  set_closed_mm(0.0, 4)
  set_open_mm(50.0, 4)
  rq_current_limit_enabled = False
  while(not rq_gripper_communication_thread_started):
      sync()
  end
  # end: URCap Installation Node
  
  
  ##########################################################################################
  #####Commands below will be executed, most common commands are listed below          #####
  #####Please refer to the instruction manual for a complete list of available commands#####
  #####remove the "#" sign before a command to execute the command                     #####
  #####Or add the "#" sign to ignore the comman                                        #####
  ##### Used as is, the gripper will open and close                                    #####
  ##########################################################################################
  
  
#######################################  
##### Here are the vacuum commands#####
#######################################
  
  
 #####Used as is, the following command will use default value
#  rq_vacuum_grip ()
#####Following command allows the Vacuum Gripper to pick an object. The default values are :advanced_mode=False, maximum_vacuum=60, minimum_vacuum=40, timeout_ms=3000, wait_for_object_detected=True, gripper_socket=" 1"
#  rq_vacuum_grip (  False  ,60,40,3000,  True  ,1)
#  rq_vacuum_release ()
#####Following command allows the Vacuum Gripper to grip an object. The default values are : advanced_mode=False, shutoff_distance_cm=5, wait_for_object_released=True, gripper_socket=" 1"
#  rq_vacuum_release (  False  ,5,  True  ,1)
#  rq_is_vacuum_obj_detected ()
  
##########################################
#####  Here are the Grippers commands#####
##########################################
  
#  rq_reset ()
#  rq_activate_and_wait ()
#  rq_set_speed_norm (100)
#  rq_set_force_norm (100)
#  rq_open_and_wait ()
#  rq_close()
#  rq_close_and_wait ()
#  rq_open()
#  rq_is_object_detected ()
#  rq_move_and_wait_norm (75)
#  rq_current_pos_norm ()
#  rq_current_pos_mm ()
#  rq_current_pos_inches ()
#  rq_move_and_wait_mm (25)
#  rq_current_pos_inches (1)

"""

FUNC_HEADER = """
def f():
"""

FUNC_FOOTER = """
end
"""

GRIPPER_ACTIVATE_COMMAND = "rq_activate_and_wait()"
GRIPPER_OPEN_COMMAND = "rq_open_and_wait()"
GRIPPER_CLOSE_COMMAND = "rq_close_and_wait()"

TABLE_A_ORIENTATION = (0.000, 3.148, 0.000)
TABLE_B_ORIENTATION = (2.224, -2.224, 0.000)

A_1_Z_MAX = 0.450
A_2_Z_MAX = 0.800
A_3_Z_MAX = 0.800
A_4_Z_MAX = 0.450

B_1_Z_MAX = 0.800
B_2_Z_MAX = 0.800
B_3_Z_MAX = 0.450
B_4_Z_MAX = 0.450

HOME_POSITION_JOIN_STATES = [
    radians(-90),
    radians(-90),
    radians(-90),
    radians(-90),
    radians(90),
    radians(0)
]

BOX_SIDE = 0.110
EDGE_OFFSET = BOX_SIDE #abs(WAYPOINTS["b_1"].X - WAYPOINTS["box_1"].X)
