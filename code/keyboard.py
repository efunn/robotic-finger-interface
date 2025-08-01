from dynamixel_sdk import *
import numpy as np
import os, yaml, multiprocessing

# unit conversions
RAW_TO_DEG    = 0.087891 # conversion from raw units to degrees
DEG_TO_RAW    = 1/RAW_TO_DEG # conversion from degrees to raw units 
RAW_TO_RPM    = 0.22888 # conversion from raw units to RPM
RPM_TO_RAW    = 1/RAW_TO_RPM # conversion from RPM to raw units 
MAX_ANGLE     = 270.0
HALF_VELOCITY = 0x7fffffff
MAX_VELOCITY  = 0xffffffff+1
MAX_TIME = int((0xffff-1)/2)
MAX_TIME_CORRECT = MAX_TIME+1

########################
# addresses and lengths
########################
# basic addresses
PROTOCOL_VERSION      = 2.0 # Dynamixel Protocol 2.0
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
LEN_POSITION          = 4
LEN_VELOCITY          = 4
ADDR_P_GAIN           = 84
ADDR_I_GAIN           = 82
ADDR_D_GAIN           = 80
ADDR_GOAL_CURRENT     = 102
LEN_GAIN              = 2
ADDR_REALTIME_TICK    = 120
LEN_TIME              = 2

# syncread/syncwrite addresses
ADDR_INDIRECT_START   = 168
LEN_ADDR_INDIRECT     = 2
IND_REALTIME_TICK     = ADDR_INDIRECT_START
IND_PRESENT_POSITION  = IND_REALTIME_TICK+LEN_ADDR_INDIRECT*LEN_TIME
IND_PRESENT_VELOCITY  = IND_PRESENT_POSITION+LEN_ADDR_INDIRECT*LEN_POSITION
IND_P_GAIN            = IND_PRESENT_VELOCITY+LEN_ADDR_INDIRECT*LEN_VELOCITY
IND_I_GAIN            = IND_P_GAIN+LEN_ADDR_INDIRECT*LEN_GAIN
IND_D_GAIN            = IND_I_GAIN+LEN_ADDR_INDIRECT*LEN_GAIN
IND_GOAL_CURRENT      = IND_D_GAIN+LEN_ADDR_INDIRECT*LEN_GAIN

ADDR_DICTS = {'realtime_tick':{'addr':ADDR_REALTIME_TICK, 'ind':IND_REALTIME_TICK, 'len':LEN_TIME},
              'present_position':{'addr':ADDR_PRESENT_POSITION, 'ind':IND_PRESENT_POSITION, 'len':LEN_POSITION},
              'present_velocity':{'addr':ADDR_PRESENT_VELOCITY, 'ind':IND_PRESENT_VELOCITY, 'len':LEN_VELOCITY},
              'p_gain':{'addr':ADDR_P_GAIN, 'ind':IND_P_GAIN, 'len':LEN_GAIN},
              'i_gain':{'addr':ADDR_I_GAIN, 'ind':IND_I_GAIN, 'len':LEN_GAIN},
              'd_gain':{'addr':ADDR_D_GAIN, 'ind':IND_D_GAIN, 'len':LEN_GAIN},
              'goal_current':{'addr':ADDR_GOAL_CURRENT, 'ind':IND_GOAL_CURRENT, 'len':LEN_GAIN}
             }

# syncread/syncwrite data
DATA_INDIRECT_START   = 208
ADDR_SYNCREAD_START   = DATA_INDIRECT_START
ADDR_TIME_DATA        = ADDR_SYNCREAD_START
ADDR_POS_DATA         = ADDR_TIME_DATA+LEN_TIME
ADDR_VEL_DATA         = ADDR_POS_DATA+LEN_POSITION
LEN_SYNCREAD          = LEN_TIME+LEN_POSITION+LEN_VELOCITY
LEN_SYNCREAD_POS_ONLY = LEN_TIME+LEN_POSITION
ADDR_SYNCWRITE_START  = ADDR_SYNCREAD_START+LEN_SYNCREAD
LEN_SYNCWRITE         = 4*LEN_GAIN

# keyboard class to run in parent process
class KeyboardWrapper(object):
    def __init__(self, config_fname='keyboard-replay'):
        # load config
        self.config_dir = os.path.join('config',config_fname+'.yml')
        try:
            with open(self.config_dir) as f:
                self.config = yaml.load(f, Loader=yaml.FullLoader)
        except:
            print('Configuration file '+self.args.config+'.yml not found')
            sys.exit(1)

        # fingers and positions
        self.map_to_screen = self.config['map_to_screen']
        self.num_fingers = len(self.map_to_screen)
        self.all_pos = multiprocessing.Array('f',self.num_fingers)

        # command logic
        self.command_pipe_recv, self.command_pipe_send = multiprocessing.Pipe(duplex=False)
        self.valid_commands = self.config['commands']

        # start async keyboard process
        wait_for_start = multiprocessing.Event()
        self.keyboard_process = multiprocessing.Process(target=main_keyboard_loop,
            args=(self.config, self.all_pos, self.command_pipe_recv, wait_for_start))
        self.keyboard_process.start()
        wait_for_start.wait()

        # initialize basic state, turn on servos
        self.send_command('torque_on')
        self.send_command('mode_idle_compliant')

    def send_command(self, full_command):
        if type(full_command) == str:
            command = full_command
        elif type(full_command) == dict:
            command = full_command['command']
        else:
            command = ''
        if command in self.valid_commands:
            self.command_pipe_send.send(full_command)

    def shutdown(self):
        # put in safe control range for next startup
        self.send_command('mode_idle_compliant')
        self.send_command('torque_off')
        self.send_command('shutdown')
        self.keyboard_process.join()

# keyboard class to run in child process
class KeyboardAsync(object):
    def __init__(self, config_object, all_pos, command_pipe_recv):
        # load config
        self.config = config_object

        # import variables
        self.port_name = self.config['port']
        self.baudrate = self.config['baudrate']
        self.rh_ids = self.config['rh_ids']
        self.lh_ids = self.config['lh_ids']
        self.all_ids = self.rh_ids + self.lh_ids
        self.num_fingers = len(self.all_ids)
        self.mirror_map_rh = self.config['mirror_map_rh']
        self.mirror_map_lh = self.config['mirror_map_lh']
        self.map_to_screen = self.config['map_to_screen']
        self.press_angle = self.config['press_angle']
        self.max_angle = MAX_ANGLE
        self.neutral_angle = self.max_angle - self.press_angle
        self.stiff_params = self.config['stiff_params']
        self.compliant_params = self.config['compliant_params']
        self.velocity_gain = self.config['velocity_gain']
        self.valid_commands = self.config['commands']

        # serial connection
        self.portHandler = PortHandler(self.port_name)
        if not(self.portHandler.openPort()):
            print("Port open failed, quitting...")
            quit()
        if not(self.portHandler.setBaudRate(self.baudrate)):
            print("Setting baudrate failed, quitting...")
            quit()
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # init variables
        self.mode = ''
        self.keyboard_running = True
        self.command_pipe_recv = command_pipe_recv
        self.all_pos = all_pos
        self.all_time = np.full(self.num_fingers, 0, dtype='i')
        self.all_vel = np.full(self.num_fingers, 0, dtype='f')

        # recording
        self.recording = False
        self.reset_recording_data()
        self.delayed_replay = False
        self.replay_started = False
        self.last_replay_time = 0
        self.next_replay_time = 0
        self.add_replay_time = 0
        self.cumulative_replay_time = 0
        self.replay_hand = 'rh'

        # convert typical params to bytes
        self.neutral_angle_bytes = deg_to_byte(self.neutral_angle)
        self.params_stiff_bytes = (convert2byte(self.stiff_params['P'])
            +convert2byte(self.stiff_params['I'])
            +convert2byte(self.stiff_params['D'])
            +convert2byte(self.stiff_params['current']))
        self.params_compliant_bytes = (convert2byte(self.compliant_params['P'])
            +convert2byte(self.compliant_params['I'])
            +convert2byte(self.compliant_params['D'])
            +convert2byte(self.compliant_params['current']))

        # init indirect addresses
        # SyncRead: [REALTIME_TICK[2], PRESENT_POSITION[4], PRESENT_VELOCITY[4]]
        # SyncWrite: [P_GAIN[2], I_GAIN[2], D_GAIN[2], GOAL_CURRENT[2]]
        for dxl_id in self.all_ids:
            for addr_set in ADDR_DICTS:
                addr_dict = ADDR_DICTS[addr_set]
                for addr in range(addr_dict['len']):
                    self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id,
                        addr_dict['ind']+LEN_ADDR_INDIRECT*addr, addr_dict['addr']+addr)

        # init syncread/syncwrite
        self.all_syncread = GroupSyncRead(self.portHandler, self.packetHandler,
            ADDR_SYNCREAD_START, LEN_SYNCREAD)
        self.all_syncwrite_pos_neutral = GroupSyncWrite(self.portHandler, self.packetHandler,
            ADDR_GOAL_POSITION, LEN_POSITION)
        self.rh_syncwrite_pos = GroupSyncWrite(self.portHandler, self.packetHandler,
            ADDR_GOAL_POSITION, LEN_POSITION)
        self.lh_syncwrite_pos = GroupSyncWrite(self.portHandler, self.packetHandler,
            ADDR_GOAL_POSITION, LEN_POSITION)
        self.all_syncwrite_gain_stiff = GroupSyncWrite(self.portHandler, self.packetHandler,
            ADDR_SYNCWRITE_START, LEN_SYNCWRITE)
        self.rh_syncwrite_gain_stiff = GroupSyncWrite(self.portHandler, self.packetHandler,
            ADDR_SYNCWRITE_START, LEN_SYNCWRITE)
        self.lh_syncwrite_gain_stiff = GroupSyncWrite(self.portHandler, self.packetHandler,
            ADDR_SYNCWRITE_START, LEN_SYNCWRITE)
        self.all_syncwrite_gain_compliant = GroupSyncWrite(self.portHandler, self.packetHandler,
            ADDR_SYNCWRITE_START, LEN_SYNCWRITE)
        self.rh_syncwrite_gain_compliant = GroupSyncWrite(self.portHandler, self.packetHandler,
            ADDR_SYNCWRITE_START, LEN_SYNCWRITE)
        self.lh_syncwrite_gain_compliant = GroupSyncWrite(self.portHandler, self.packetHandler,
            ADDR_SYNCWRITE_START, LEN_SYNCWRITE)

        for dxl_id in self.all_ids:
            self.all_syncread.addParam(dxl_id)
            self.all_syncwrite_pos_neutral.addParam(dxl_id, self.neutral_angle_bytes)
            self.all_syncwrite_gain_stiff.addParam(dxl_id, self.params_stiff_bytes)
            self.all_syncwrite_gain_compliant.addParam(dxl_id, self.params_compliant_bytes)
        for dxl_id in self.rh_ids:
            self.rh_syncwrite_pos.addParam(dxl_id, self.neutral_angle_bytes)
            self.rh_syncwrite_gain_stiff.addParam(dxl_id, self.params_stiff_bytes)
            self.rh_syncwrite_gain_compliant.addParam(dxl_id, self.params_compliant_bytes)
        for dxl_id in self.lh_ids:
            self.lh_syncwrite_pos.addParam(dxl_id, self.neutral_angle_bytes)
            self.lh_syncwrite_gain_stiff.addParam(dxl_id, self.params_stiff_bytes)
            self.lh_syncwrite_gain_compliant.addParam(dxl_id, self.params_compliant_bytes)

        # init values from servos
        self.all_syncread.fastSyncRead()

    def enable_torque_all(self):
        for dxl_id in self.all_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)

    def disable_torque_all(self):
        for dxl_id in self.all_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)

    def handle_command(self):
        full_command = self.command_pipe_recv.recv()
        if type(full_command) == str:
            command = full_command
        elif type(full_command) == dict:
            command = full_command['command']
            command_data = full_command['data']
        else:
            command = ''
        if command in self.valid_commands:
            # set mode if command is a mode switch
            if command.startswith('mode'):
                self.mode = command.rsplit('mode_')[-1]
            # run appropriate actions for command
            if command == 'shutdown':
                self.shutdown()
            elif command == 'torque_on':
                self.enable_torque_all()
            elif command == 'torque_off':
                self.disable_torque_all()
            elif command == 'mode_idle_stiff':
                self.all_syncwrite_pos_neutral.txPacket()
                self.all_syncwrite_gain_stiff.txPacket()
            elif command == 'mode_idle_compliant':
                self.all_syncwrite_pos_neutral.txPacket()
                self.all_syncwrite_gain_compliant.txPacket()
            elif command == 'mode_action_normal_rh':
                self.all_syncwrite_pos_neutral.txPacket()
                self.rh_syncwrite_gain_compliant.txPacket()
                self.lh_syncwrite_gain_stiff.txPacket()
            elif command == 'mode_action_normal_lh':
                self.all_syncwrite_pos_neutral.txPacket()
                self.rh_syncwrite_gain_stiff.txPacket()
                self.lh_syncwrite_gain_compliant.txPacket()
            elif command == 'mode_action_mirror_rh':
                self.all_syncwrite_pos_neutral.txPacket()
                self.rh_syncwrite_gain_compliant.txPacket()
                self.lh_syncwrite_gain_stiff.txPacket()
            elif command == 'mode_action_mirror_lh':
                self.all_syncwrite_pos_neutral.txPacket()
                self.rh_syncwrite_gain_stiff.txPacket()
                self.lh_syncwrite_gain_compliant.txPacket()
            elif command == 'start_recording':
                self.reset_recording_data()
                self.recording = True
            elif command == 'stop_recording':
                self.prep_for_replay(command_data)
                self.recording = False
            elif command == 'start_delayed_replay_rh':
                self.all_syncwrite_pos_neutral.txPacket()
                self.all_syncwrite_gain_stiff.txPacket()
                self.delayed_replay = True
                self.replay_hand = 'rh'
            elif command == 'start_delayed_replay_lh':
                self.all_syncwrite_pos_neutral.txPacket()
                self.all_syncwrite_gain_stiff.txPacket()
                self.delayed_replay = True
                self.replay_hand = 'lh'
            elif command == 'stop_replay':
                self.delayed_replay = False
                self.replay_started = False
            else:
                pass

    def reset_recording_data(self):
        self.recorded_time_ms = []
        self.recorded_pos_deg = {}
        self.recorded_vel_rpm = {}
        for dxl_id in self.all_ids:
            self.recorded_pos_deg[dxl_id] = []
            self.recorded_vel_rpm[dxl_id] = []

    def record_frame(self):
        self.recorded_time_ms.append(self.all_syncread.getData(self.all_ids[0],ADDR_TIME_DATA,LEN_TIME))
        for dxl_id in self.all_ids:
            self.recorded_pos_deg[dxl_id].append(raw_to_deg(self.all_syncread.getData(dxl_id,ADDR_POS_DATA,LEN_POSITION)))
            self.recorded_vel_rpm[dxl_id].append(raw_to_rpm(self.all_syncread.getData(dxl_id,ADDR_VEL_DATA,LEN_VELOCITY)))

    def prep_for_replay(self, correct_ms=0):
        # put into numpy array for easy math
        self.replay_time = np.array(self.recorded_time_ms)
        # correct for motor clock wraparound at MAX_TIME (~32 sec)
        max_time_loops = np.where(np.diff(self.replay_time)<0)
        if len(max_time_loops[0])>0:
            for time_fix in max_time_loops[0]:
                self.replay_time[(time_fix+1):] += MAX_TIME_CORRECT
        self.replay_time -= self.replay_time[0]
        # correct time for delayed start
        self.replay_time = self.replay_time[self.replay_time>=correct_ms]
        self.replay_time -= self.replay_time[0]
        # correct positions and velocities for delayed start
        replay_len_frames = len(self.replay_time)
        for dxl_id in self.all_ids:
            self.recorded_pos_deg[dxl_id] = self.recorded_pos_deg[dxl_id][-replay_len_frames:]
            self.recorded_vel_rpm[dxl_id] = self.recorded_vel_rpm[dxl_id][-replay_len_frames:]
        # indicate when replay should end
        self.replay_end_time = self.replay_time[-1]
        self.replay_started = False

    def check_run_active(self):
        if self.mode == 'action_mirror_rh':
            self.assign_mirror_map('rh')
        elif self.mode == 'action_mirror_lh':
            self.assign_mirror_map('lh')
        else:
            pass

    def check_recording_replay(self):
        if self.recording:
            self.record_frame()
        if self.delayed_replay:
            if not(self.replay_started):
                self.replay_started = True
                self.cumulative_replay_time = 0
                self.last_replay_time = self.all_syncread.getData(self.all_ids[0],ADDR_TIME_DATA,LEN_TIME)
            else:
                self.last_replay_time = self.next_replay_time
            self.next_replay_time = self.all_syncread.getData(self.all_ids[0],ADDR_TIME_DATA,LEN_TIME)
            self.add_replay_time = self.next_replay_time-self.last_replay_time
            if self.add_replay_time < 0:
                self.add_replay_time += MAX_TIME_CORRECT
            self.cumulative_replay_time += self.add_replay_time
            self.assign_delayed_replay_map(self.cumulative_replay_time, self.replay_hand)
            if self.cumulative_replay_time >= self.replay_end_time:
                self.delayed_replay = False
                self.replay_started = False

    def assign_delayed_replay_map(self, timestamp, hand='rh'):
        if hand == 'rh':
            for dxl_id in self.mirror_map_rh:
                new_angle = np.interp(timestamp, self.replay_time, self.recorded_pos_deg[dxl_id])
                new_velocity = np.interp(timestamp, self.replay_time, self.recorded_vel_rpm[dxl_id])
                update_angle = self.mirror_angle(new_angle, new_velocity)
                self.lh_syncwrite_pos.changeParam(self.mirror_map_rh[dxl_id],
                    deg_to_byte(update_angle))
            self.lh_syncwrite_pos.txPacket()
        elif hand == 'lh':
            for dxl_id in self.mirror_map_lh:
                new_angle = np.interp(timestamp, self.replay_time, self.recorded_pos_deg[dxl_id])
                new_velocity = np.interp(timestamp, self.replay_time, self.recorded_vel_rpm[dxl_id])
                update_angle = self.mirror_angle(new_angle, new_velocity)
                self.rh_syncwrite_pos.changeParam(self.mirror_map_lh[dxl_id],
                    deg_to_byte(update_angle))
            self.rh_syncwrite_pos.txPacket()
        else:
            pass

    def assign_mirror_map(self, hand='rh'):
        if hand == 'rh':
            for dxl_id in self.mirror_map_rh:
                new_angle = raw_to_deg(self.all_syncread.getData(dxl_id,ADDR_POS_DATA,LEN_POSITION))
                new_velocity = raw_to_rpm(self.all_syncread.getData(dxl_id,ADDR_VEL_DATA,LEN_VELOCITY))
                update_angle = self.mirror_angle(new_angle, new_velocity)
                self.lh_syncwrite_pos.changeParam(self.mirror_map_rh[dxl_id],
                    deg_to_byte(update_angle))
            self.lh_syncwrite_pos.txPacket()
        elif hand == 'lh':
            for dxl_id in self.mirror_map_lh:
                new_angle = raw_to_deg(self.all_syncread.getData(dxl_id,ADDR_POS_DATA,LEN_POSITION))
                new_velocity = raw_to_rpm(self.all_syncread.getData(dxl_id,ADDR_VEL_DATA,LEN_VELOCITY))
                update_angle = self.mirror_angle(new_angle, new_velocity)
                self.rh_syncwrite_pos.changeParam(self.mirror_map_lh[dxl_id],
                    deg_to_byte(update_angle))
            self.rh_syncwrite_pos.txPacket()
        else:
            pass

    def mirror_angle(self, new_angle, new_velocity):
        return min(self.max_angle,max(self.neutral_angle,new_angle+new_velocity*self.velocity_gain))

    def shutdown(self):
        self.keyboard_running = False

def main_keyboard_loop(config_object, all_pos, command_pipe_recv, wait_for_start):
    # create keyboard object inside child process
    kb = KeyboardAsync(config_object, all_pos, command_pipe_recv)
    wait_for_start.set()

    # init time
    next_time = [kb.all_syncread.getData(dxl_id,ADDR_TIME_DATA,LEN_TIME) for dxl_id in kb.all_ids]

    # main keyboard loop until shutdown called
    while kb.keyboard_running:
        while kb.command_pipe_recv.poll():
            kb.handle_command()
        kb.all_syncread.fastSyncRead()
        kb.all_time[:] = [kb.all_syncread.getData(dxl_id,ADDR_TIME_DATA,LEN_TIME) for dxl_id in kb.all_ids]
        last_time = next_time
        next_time = [kb.all_syncread.getData(dxl_id,ADDR_TIME_DATA,LEN_TIME) for dxl_id in kb.all_ids]
        kb.all_time[:] = [next_time[idx] - last_time[idx] for idx in range(kb.num_fingers)]
        kb.all_pos[:] = [raw_to_deg(kb.all_syncread.getData(dxl_id,ADDR_POS_DATA,LEN_POSITION)) for dxl_id in kb.all_ids]
        kb.all_vel[:] = [raw_to_rpm(kb.all_syncread.getData(dxl_id,ADDR_VEL_DATA,LEN_VELOCITY)) for dxl_id in kb.all_ids]
        kb.check_run_active()
        kb.check_recording_replay()
    # handle any remaining cleanup/shutdown commands
    while kb.command_pipe_recv.poll():
        kb.handle_command()
    kb.portHandler.closePort()

###############################
# utility/conversion functions
###############################
def raw_to_deg(input_raw):
    return input_raw*RAW_TO_DEG

def deg_to_raw(input_deg):
    return round(input_deg*DEG_TO_RAW)

def deg_to_byte(input_deg):
    return convert4byte(round(input_deg*DEG_TO_RAW))

def raw_to_rpm(input_raw):
    if input_raw > HALF_VELOCITY:
        return (input_raw - MAX_VELOCITY)*RAW_TO_RPM
    else:
        return input_raw*RAW_TO_RPM

def rpm_to_raw(input_rpm):
    return round(input_rpm*RPM_TO_RAW)

def convert2byte(data):
    return [DXL_LOBYTE(data), DXL_HIBYTE(data)]

def convert4byte(data):
    return [DXL_LOBYTE(DXL_LOWORD(data)),
            DXL_HIBYTE(DXL_LOWORD(data)),
            DXL_LOBYTE(DXL_HIWORD(data)),
            DXL_HIBYTE(DXL_HIWORD(data))]
