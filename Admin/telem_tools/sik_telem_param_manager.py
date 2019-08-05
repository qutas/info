#!/usr/bin/env python2
#
# Serial firmware uploader for the SiK bootloader
#

import sys, argparse, binascii, serial, glob, time

def init_port(portname, baudrate=57600, use_mavlink=False, mavport=0):
	print("Connecting to %s" % portname)
	port = None

	if use_mavlink:
		from pymavlink import mavutil
		port = mavutil.MavlinkSerialPort(portname, baudrate, devnum=mavport)
	else:
		port = serial.Serial(portname, baudrate, timeout=3)

	return port

def read_response(port):
	r = p.readline().rstrip()
	if not r:
		raise(ValueError('Timeout on response read'))

	return r

def send_command(port, cmd):
	p.write("%s\r\n" % str(cmd))
	p.readline()

def send_command_ok(port, cmd):
	send_command(port,cmd)
	res = port.readline().rstrip()
	#print("req: %s" % cmd)
	#print("res: %s" % res)
	return res == 'OK'

def enter_command(port):
	port.write(b'+++')
	return port.readline().rstrip() == 'OK'

def close_port(port):
	if port:
		if port.is_open:
			send_command(p,'ATO')
			port.close()

def close_prog(port,ec=0):
	close_port(port)
	sys.exit(ec)

def make_param(ind,name,val):
	return {
		"ind": int(ind),
		"name": str(name),
		"value": int(val)
	}


def param_entry(line):
	p = line.rstrip()

	pc = p.split(':',1)
	pi = str(pc[0])

	if len(pc) > 1:
		pe = pc[1].split('=',1)
		pn = str(pe[0])
		pv = int(pe[1])

		return(make_param(pi.split('S')[1],pn,pv))
	else:
		return(make_param(-1,pi,0))

def check_params(params1,params2):
	success = False

	if (len(params1) == len(params2)) and (len(params1) > 1):
		p1ver_l = params1[0]['name'].split(' ')
		p1ver = p1ver_l[0] + 'on' + p1ver_l[1]
		p2ver_l = params2[0]['name'].split(' ')
		p2ver = p2ver_l[0] + 'on' + p2ver_l[1]

		if (p1ver == p2ver):
			i = 1
			while i < len(params1):
				if params1[i]['name'] != params2[i]['name']:
					print("Error checking parameter S%i: %s | %s" % (i,params1[i]['name'],params2[i]['name']))
					break
				i += 1

			if i == len(params1):
				# We made it to the end without breaking
				success = True

	return success

def do_param_logic(port,new_params,backup_file):
	if not enter_command(port):
		print("Failed to contact bootloader")
		close_prog(port,ec=1)

	send_command(port,'ATI')
	ver = read_response(port)
	print("Firmware version: %s" % ver)

	old_params = [param_entry(ver)]

	send_command(port,'ATI5')
	while True:
		try:
			old_params.append(param_entry(read_response(port)))
		except ValueError as e:
			#print("Error: %s" % e) #XXX: means we have read all the parameters here
			break

	print("Current parameters:")
	with open(backup_file,'w') as bf:
		for param in old_params:
			if param['ind'] == -1:
				# This is the version parameter
				bf.write(str(param['name']) + '\n')
			else:
				# This is a normal parameter
				p_str = "S%i:%s=%i" % (param['ind'],param['name'],param['value'])
				bf.write(p_str + '\n')
				print('    ' + p_str)

	print("Wrote backup file to: %s" % backup_file)

	if len(new_params) > 0:
		params_match = False
		try:
			params_match = check_params(new_params, old_params)
		except Exception as e:
			print("Exception %s" % e)

		if params_match:
			print("Writing new parameters")
			try:
				#TODO: Could check here to skip already-set parameters
				for param in new_params:
					# We write all params except the 'FORMAT' parameter (sets bitwise memory format, should only be set by firmware)
					if (param['name'] != 'FORMAT') and (param['ind'] >= 0):
						if not send_command_ok(port,"ATS%i=%i" % (param['ind'],param['value'])):
							raise(RuntimeError("Param set command did not return OK"))

				if not send_command_ok(port,'AT&W'):
					raise(RuntimeError("Param write command did not return OK"))

				send_command(port,'ATZ')
			except RuntimeError as e:
				print("Error: %s" % e)
		else:
			print("Error: Parameter version/count mismatch")
	else:
		print('Read only, not writing parameters')

	print("Finished process for port %s" % port.name)
	close_port(port)


if __name__ == '__main__':
	# Parse commandline arguments
	parser = argparse.ArgumentParser(description="Firmware uploader for the SiK radio system.")
	parser.add_argument("--baudrate", type=int, default=115200, help='baud rate')
	parser.add_argument("--mavlink", action='store_true', default=False, help='update over MAVLink')
	parser.add_argument("--mavport", type=int, default=0, help='MAVLink port number')
	parser.add_argument('--port', required=True, action="store", help="port to upload to")
	parser.add_argument('--backupfile', type=str, default="/tmp/sik_radio.params.old", help="File to save the old parameters to")
	parser.add_argument('--newparams', type=str, default="", help="New parameters to be written. If not specified, parameters will only be read to backup file")
	args = parser.parse_args()

	# Load the firmware file
	parameters = []
	if args.newparams:
		with open(args.newparams) as fp:
			for line in fp:
				parameters.append(param_entry(line))

	ports = glob.glob(args.port)
	if not ports:
	        print("No matching ports for %s" % args.port)
	        sys.exit(1)

	# Connect to the device and identify it
	open_counter = 0
	for port in glob.glob(args.port):
		print("uploading to port %s" % port)
		p = init_port(port, baudrate=args.baudrate, use_mavlink=args.mavlink,mavport=args.mavport)
		if p:
			backup_file = args.backupfile
			if open_counter >= 1:
				backup_file += ".%i" % open_counter

			do_param_logic(p,parameters,backup_file)
