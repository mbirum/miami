import subprocess
import rtmidi

class MIDIInterface:
    midiout = None
    port_index = -1
    channel_hex_base = 177

    def __init__(self):
        self.check_device()

    def check_device(self):
        self.midiout = rtmidi.MidiOut()
        available_ports = self.midiout.get_ports()

        for i in range(len(available_ports)):
            if "rtpmidi" in available_ports[i]:
                try:
                    self.port_index = i
                    self.midiout.open_port(i)
                    print("rtpmidi port registered")
                except:
                    raise Exception('rtpmidi port could not be opened')
                
        if self.port_index == -1:
            raise Exception('rtpmidi port not available')

    def send_cc_message(self, cc, channel, value):
        if self.port_index != -1:
            hex_str = hex(self.channel_hex_base + channel)
            channel_hex = int(hex_str, 16)
            self.midiout.send_message([channel_hex, cc, value])
