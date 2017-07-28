import serial
import time

x=1
y=2
z=3

class MotorOutOfRangeError(Exception):
    pass

class PhyMotionController():
    def __init__(self, comport, timeout=0.5, axes_config=None):
        self.timeout = timeout
        self.movement_boundaries = ((0,0), (0,0), (0, 0), None, None, None, None)
        if type(comport) is int:
            self.comport = 'COM' + str(comport)
        else:
            self.comport = comport
        self.s = serial.Serial(self.comport, 115200, timeout=self.timeout)
        if self.s is None:
            raise Exception('Could not connect to PhyMotion')
        self.configure_controller()
        self.position_vec = self.number_of_axis * [None]
        self.position_history = [[] for i in range(self.number_of_axis)]
        self.last_known_position = [[] for i in range(self.number_of_axis)]
        self.read_multiple_position()
        self.last_connection_time = time.time()
        self.step_res_dict = {'0': 1, '1': 2, '2': 2.5, '3': 4, '4': 5, '5': 8, '6': 10, '7': 16, '8': 20, '9': 32,
                              '10': 64, '11': 128, '12': 256, '13': 512}
        self.motor_steps = 200
        self._axes_config = []
        for i in range(self.number_of_axis):
            self._axes_config.append({'axis': i + 1})
        if axes_config is not None:
            self.axes_config = axes_config

    def init_xyz_xrayoptics(self):
        config_axis1 = {'axis': 1, 'active': True, 'moving': False, 'frequency': int(self.send_command('1.1P14R')), 'rampfrequency': int(self.send_command('1.1P15R')),
                        'stepresolution': 10, 'laufcurrent': 60}
        config_axis2 = {'axis': 2, 'active': True, 'moving': False, 'frequency': int(self.send_command('2.1P14R')), 'rampfrequency': int(self.send_command('2.1P15R')),
                        'stepresolution': 10, 'laufcurrent': 60}
        config_axis3 = {'axis': 3, 'active': True, 'moving': False, 'frequency': int(self.send_command('3.1P14R')), 'rampfrequency': int(self.send_command('3.1P15R')),
                        'stepresolution': 9, 'laufcurrent': 60}
        config_axis4 = {'axis': 4, 'active': True, 'moving': False, 'frequency': int(self.send_command('4.1P14R')), 'rampfrequency': int(self.send_command('4.1P15R')),
                        'stepresolution': 10, 'laufcurrent': 60}
        config_axis5 = {'axis': 5, 'active': True, 'moving': False, 'frequency': int(self.send_command('5.1P14R')), 'rampfrequency': int(self.send_command('5.1P15R')),
                        'stepresolution': 9, 'laufcurrent': 60}
        self.axes_config = [config_axis1, config_axis2, config_axis3]

    def configure_controller(self):
        self.number_of_axis = int(self.send_command('S')[0])  # get number of axis0

    @property
    def axes_config(self):
        return self._axes_config

    @axes_config.setter
    def axes_config(self, input_list):
        def make_config(el):
            self.set_config_for_axis(**el)
            self._axes_config[el['axis'] - 1].update(el)

        if type(input_list) == list:
            for el in input_list:
                make_config(el)
                time.sleep(0.1)
        elif type(input_list) == dict:
            make_config(input_list)
        else:
            raise ValueError('Expected list of dict or dict')
        self.update_boundaries()

    def set_config_for_axis(self, axis=None, active=None, moving=None, position=None, frequency=None,
                            rampfrequency=None, stepresolution=None, laufcurrent=None):
        command_string = ''
        if axis is None:
            raise ValueError('axis must be supplied in axis_config')
        if position is not None:
            command_string += str(axis) + '.1P20S{0:+d} '.format(position)
        if active is not None:
            if active:
                command_string += str(axis) + '.1MA '
            else:
                command_string += str(axis) + '.1MD '
        if moving is not None:
            pass
        if frequency is not None:
            command_string += str(axis) + '.1P14S{0:+d} '.format(frequency)
        if rampfrequency is not None:
            command_string += str(axis) + '.1P15S{0:+d} '.format(rampfrequency)
        if stepresolution is not None:
            command_string += str(axis) + '.1P45S{0:+d} '.format(stepresolution)
        if laufcurrent is not None:
            command_string += str(axis) + '.1P41S{0:+d} '.format(laufcurrent)
        if len(command_string) > 0:
            self.send_command(command_string)
            print('---Axis ' + str(axis) + ' Controller Update Successful---')
        else:
            return

    def update_boundaries(self):
        axis1 = (-20000 / self.calc_step_len(x), 20000 / self.calc_step_len(x))
        axis2 = (-30000 / self.calc_step_len(y), 30000 / self.calc_step_len(y))
        axis3 = (-10000 / self.calc_step_len(z), 10000 / self.calc_step_len(z))
        self.movement_boundaries = (axis1, axis2, axis3, None, None, None, None)  # copperL, copperR, ZaberL, ZaberR

    def my_readline(self, delimiter=b'\x03'):
        read_byte = self.s.read(1)
        read_bytes = read_byte
        while read_byte is not bytes(delimiter) and self.s.in_waiting != 0:
            read_byte = self.s.read(1)
            read_bytes += read_byte
        return read_bytes

    def send_command(self, command):
        send_str = '\x020' + command + ':XX\x03'
        self.s.write(send_str.encode('ASCII'))
        answer = self.receive_command()
        return self.translate_input(answer)

    def receive_command(self, timeout=None):
        if timeout is None:
            timeout = self.timeout
        start_time = time.time()
        while time.time() - start_time < timeout and self.s.in_waiting == 0:
            time.sleep(0.01)
        # typical response time is 0.03 seconds
        time.sleep(0.02)  # important to receive the full string
        if self.s.in_waiting == 0:
            return
        read_str = self.s.read(self.s.in_waiting)
        cut_str = read_str.split(b'\x03')[0].decode('ASCII')
        return cut_str

    def translate_input(self, answer):
        if len(answer) < 3:
            Exception('Could not translate Phymotion input: string is too short')
        elif answer[1] == '\x06':
            return answer[2:].split(':')[0]
        else:
            raise Exception('Command not recognized')

    def read_position(self, axis):
        self.block_movement(axis=axis)
        if self._axes_config[axis - 1]['moving'] == False:
            pos = int(self.send_command(str(axis) + '.1P20R'))
            self.position_vec[axis - 1] = pos
            pos_micrometers = pos * self.calc_step_len(axis)
            return pos_micrometers

    def set_position(self, axis, position):
        if self._axes_config[axis - 1]['moving'] is not True:
            pos_micrometers = int(position)
            pos = pos_micrometers / self.calc_step_len(axis)
            self.send_command(str(axis) + '.1P20S' + str(pos))
            return pos_micrometers
        else:
            print('Cannot set position of axis since it is moving')

    def set_as_zero_position(self, axis):
        if self._axes_config[axis - 1]['moving'] == False:
            self.set_position(axis=axis, position=0)
            file_edit_position = open('previous_position_' + str(axis) + '.txt', 'w')
            file_edit_position.write(str(0))
            file_edit_position.close()
            file_new_position = open('previous_position_' + str(axis) + '.txt', 'r')
            new_position = file_new_position.read()
            file_new_position.close()
            print('Zero position for axis ' + str(axis) + ' set')
        else:
            print('Cannot set zero position of axis since it is moving')

    def set_as_grand_zero_position(self):
        self.set_as_zero_position(x)
        self.set_as_zero_position(y)
        self.set_as_zero_position(z)
        print('~~~Grand zero position for axes set~~~')

    def read_multiple_position(self, history_bool= True):
        self.s.reset_input_buffer()
        send_str = ''
        for i in range(1, self.number_of_axis + 1):
            send_str += str(i) + '.1P20R '
        pm_answer = self.send_command(send_str).split('\x06')
        self.position_vec = [int(i) for i in pm_answer]
        if history_bool:
            for i, pos in enumerate(self.position_vec):
                self.position_history[i].append(pos)
        return self.position_vec

    def block_movement(self, axis):
        while self.read_movement(axis=axis) == True:
            pass

    def check_boundaries(self, axis, end_position):
        if self.movement_boundaries[axis - 1] is not None:
            if not self.movement_boundaries[axis - 1][0] < end_position < self.movement_boundaries[axis - 1][1]:
                raise MotorOutOfRangeError

    def load_file(self, axis):
        file_previous_position = open('previous_position_' + str(axis) + '.txt', 'r')
        previous_position = file_previous_position.read()
        file_previous_position.close()
        print('Initial position for axis ' + str(axis) + ': ' + previous_position + ' micrometers')
        self.set_position(axis, int(previous_position))

    def save_file(self, axis):
        if self._axes_config[axis - 1]['moving'] == False:
            file_edit_position = open('previous_position_' + str(axis) + '.txt', 'w')
            file_edit_position.write(str(int(self.read_position(axis))))
            file_edit_position.close()
            file_new_position = open('previous_position_' + str(axis) + '.txt', 'r')
            new_position = file_new_position.read()
            file_new_position.close()
            print('New position for axis ' + str(axis) + ': ' + new_position + ' micrometers')

    def move_multiple_dist_rel(self, input_vec):
        command_str = ''
        for el in input_vec:
            axis = el[0]
            distance = el[1]
            self.load_file(axis=axis)
            position = (self.read_position(axis=axis)) / self.calc_step_len(axis)
            end_position = position + (int(distance / float(self.calc_step_len(axis))))
            self.check_boundaries(axis=axis, end_position=end_position)
            command_str += str(axis) + '.1{0:+d} '.format((int(distance / float(self.calc_step_len(axis)))))
            print('axis ' + str(axis) + ' moving...')
        command_str = command_str
        self.send_command(command_str)
        self.block_movement(axis=axis)
        for el in input_vec:
            axis = el[0]
            self.save_file(axis=axis)

    def move_multiple_dist_abs(self, input_vec, history_bool=True):
        old_pos = self.read_multiple_position(history_bool=history_bool)
        command_str = ''
        for el in input_vec:
            axis = el[0]
            end_distance = el[1]
            self.load_file(axis=axis)
            end_position = (int(end_distance / float(self.calc_step_len(axis))))
            self.check_boundaries(axis=axis, end_position=end_position)
            command_str += str(axis) + '.1{0:+d} '.format(int(end_position - int(old_pos[axis - 1])))
            if end_position - old_pos[axis - 1] is 0:
                print('Axis ' + str(axis) + ' is already at end position')
            else:
                print('axis ' + str(axis) + ' is moving...')
        command_str = command_str[:-1]
        self.send_command(command_str)
        self.block_movement(axis=axis)
        for el in input_vec:
            axis = el[0]
            self.save_file(axis=axis)

    def calc_step_len(self, axis):
        if axis == x or axis == y:
            dist_rev = 1000
            step_res = self._axes_config[axis - 1]['stepresolution']
            step_res_str = ''
            step_res_str += str(step_res)
            steps_rev = 200 * (self.step_res_dict[step_res_str])
            step_len = dist_rev / steps_rev
            return step_len
        else:
            dist_rev = 500
            step_res = self._axes_config[axis - 1]['stepresolution']
            step_res_str = ''
            step_res_str += str(step_res)
            steps_rev = 200 * (self.step_res_dict[step_res_str])
            step_len = dist_rev / steps_rev
            return step_len

    def stop(self, axis):
        self.send_command(str(axis) + '.1S')
        self._axes_config[axis - 1]['moving'] = False

    def stop_all(self):
        stop_command = ''
        for i in range(self.number_of_axis):
            stop_command += str(i + 1) + '.1S '
            self._axes_config[i]['moving'] = False
        self.send_command(stop_command)

    def close_connection(self):
        self.s.close()

    def read_movement(self, axis):
        mov = (self.send_command(str(axis) + '.1!=H'))
        time.sleep(0.05)
        self.last_known_position = self.read_multiple_position()
        time.sleep(0.05)
        if mov is 'E':
            self._axes_config[axis - 1]['moving'] = True
            return True
        else:
            self._axes_config[axis - 1]['moving'] = False
            return False


if __name__ == "__main__":
    phym1 = PhyMotionController('COM11')
    phym1.init_xyz_xrayoptics()