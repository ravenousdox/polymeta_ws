self.declare_parameter('mode_command', 'STABILIZE') 
self.requested_mode = self.get_parameter('mode_command').value.upper()