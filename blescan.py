# BLE iBeaconScanner based on https://github.com/adamf/BLE/blob/master/ble-scanner.py
# JCS 06/07/14

DEBUG = True
# BLE scanner based on https://github.com/adamf/BLE/blob/master/ble-scanner.py
# BLE scanner, based on https://code.google.com/p/pybluez/source/browse/trunk/examples/advanced/inquiry-with-rssi.py

# https://github.com/pauloborges/bluez/blob/master/tools/hcitool.c for lescan
# https://kernel.googlesource.com/pub/scm/bluetooth/bluez/+/5.6/lib/hci.h for opcodes
# https://github.com/pauloborges/bluez/blob/master/lib/hci.c#L2782 for functions used by lescan

# performs a simple device inquiry, and returns a list of ble advertizements 
# discovered device

# NOTE: Python's struct.pack() will add padding bytes unless you make the endianness explicit. Little endian
# should be used for BLE. Always start a struct.pack() format string with "<"

import os
import sys
import struct
import bluetooth._bluetooth as bluez
import math

LE_META_EVENT = 0x3e
LE_PUBLIC_ADDRESS=0x00
LE_RANDOM_ADDRESS=0x01
LE_SET_SCAN_PARAMETERS_CP_SIZE=7
OGF_LE_CTL=0x08
OCF_LE_SET_SCAN_PARAMETERS=0x000B
OCF_LE_SET_SCAN_ENABLE=0x000C
OCF_LE_CREATE_CONN=0x000D

LE_ROLE_MASTER = 0x00
LE_ROLE_SLAVE = 0x01

# these are actually subevents of LE_META_EVENT
EVT_LE_CONN_COMPLETE=0x01
EVT_LE_ADVERTISING_REPORT=0x02
EVT_LE_CONN_UPDATE_COMPLETE=0x03
EVT_LE_READ_REMOTE_USED_FEATURES_COMPLETE=0x04

# Advertisment event types
ADV_IND=0x00
ADV_DIRECT_IND=0x01
ADV_SCAN_IND=0x02
ADV_NONCONN_IND=0x03
ADV_SCAN_RSP=0x04


def returnnumberpacket(pkt):
	myInteger = 0
	multiple = 256
	for c in pkt:
		myInteger +=  struct.unpack("B",c)[0] * multiple
		multiple = 1
	return myInteger 

def returnstringpacket(pkt):
	myString = "";
	for c in pkt:
		myString +=  "%02x" %struct.unpack("B",c)[0]
	return myString 

def printpacket(pkt):
	for c in pkt:
		sys.stdout.write("%02x " % struct.unpack("B",c)[0])

def get_packed_bdaddr(bdaddr_string):
	packable_addr = []
	addr = bdaddr_string.split(':')
	addr.reverse()
	for b in addr: 
		packable_addr.append(int(b, 16))
	return struct.pack("<BBBBBB", *packable_addr)

def packed_bdaddr_to_string(bdaddr_packed):
	return ':'.join('%02x'%i for i in struct.unpack("<BBBBBB", bdaddr_packed[::-1]))

def hci_enable_le_scan(sock):
	hci_toggle_le_scan(sock, 0x01)

def hci_disable_le_scan(sock):
	hci_toggle_le_scan(sock, 0x00)

def hci_toggle_le_scan(sock, enable):
# hci_le_set_scan_enable(dd, 0x01, filter_dup, 1000);
# memset(&scan_cp, 0, sizeof(scan_cp));
 #uint8_t		 enable;
 #	   uint8_t		 filter_dup;
#		scan_cp.enable = enable;
#		scan_cp.filter_dup = filter_dup;
#
#		memset(&rq, 0, sizeof(rq));
#		rq.ogf = OGF_LE_CTL;
#		rq.ocf = OCF_LE_SET_SCAN_ENABLE;
#		rq.cparam = &scan_cp;
#		rq.clen = LE_SET_SCAN_ENABLE_CP_SIZE;
#		rq.rparam = &status;
#		rq.rlen = 1;

#		if (hci_send_req(dd, &rq, to) < 0)
#				return -1;
	cmd_pkt = struct.pack("<BB", enable, 0x00)
	bluez.hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, cmd_pkt)


def hci_le_set_scan_parameters(sock):
	old_filter = sock.getsockopt( bluez.SOL_HCI, bluez.HCI_FILTER, 14)

	SCAN_RANDOM = 0x01
	OWN_TYPE = SCAN_RANDOM
	SCAN_TYPE = 0x01


	
def parse_events(sock, loop_count=500):
	old_filter = sock.getsockopt( bluez.SOL_HCI, bluez.HCI_FILTER, 14)

	# perform a device inquiry on bluetooth device #0
	# The inquiry should last 8 * 1.28 = 10.24 seconds
	# before the inquiry is performed, bluez should flush its cache of
	# previously discovered devices
	flt = bluez.hci_filter_new()
	bluez.hci_filter_all_events(flt)
	bluez.hci_filter_set_ptype(flt, bluez.HCI_EVENT_PKT)
	sock.setsockopt( bluez.SOL_HCI, bluez.HCI_FILTER, flt )
	done = False
	results = []
	myFullList = []
	distanceDict = {}
	for i in range(0, loop_count):
		pkt = sock.recv(255)
		ptype, event, plen = struct.unpack("BBB", pkt[:3])
		#print "--------------" 
		if event == bluez.EVT_INQUIRY_RESULT_WITH_RSSI:
			i =0
		elif event == bluez.EVT_NUM_COMP_PKTS:
			i =0 
		elif event == bluez.EVT_DISCONN_COMPLETE:
			i =0 
		elif event == LE_META_EVENT:
			subevent, = struct.unpack("B", pkt[3])
			pkt = pkt[4:]
			if subevent == EVT_LE_CONN_COMPLETE:
				le_handle_connection_complete(pkt)
			elif subevent == EVT_LE_ADVERTISING_REPORT:
				#print "advertising report"
				num_reports = struct.unpack("B", pkt[0])[0]
				report_pkt_offset = 0
				for i in range(0, num_reports):
					if (DEBUG == True):
						deviceBltAddress = packed_bdaddr_to_string(pkt[report_pkt_offset + 3:report_pkt_offset + 9])
						#deviceUUID = returnstringpacket(pkt[report_pkt_offset -22: report_pkt_offset - 6]) 
						if deviceBltAddress == "12:3b:6a:1a:5f:4f" or deviceBltAddress == "12:3b:6a:1a:5f:46" or deviceBltAddress == "12:3b:6a:1a:62:36" or deviceBltAddress == "12:3b:6a:1a:62:38" or deviceBltAddress=="12:3b:6a:1a:5f:3a":
							#print "-------------"
							#print "\tfullpacket: ", printpacket(pkt)
							#print "\tUDID: ", printpacket(pkt[report_pkt_offset -22: report_pkt_offset - 6])
							#print "\tMAJOR: ", printpacket(pkt[report_pkt_offset -6: report_pkt_offset - 4])
							#print "\tMINOR: ", printpacket(pkt[report_pkt_offset -4: report_pkt_offset - 2])
							#print "\tMAC address: ", packed_bdaddr_to_string(pkt[report_pkt_offset + 3:report_pkt_offset + 9])
							# commented out - don't know what this byte is.  It's NOT TXPower
							txpower, = struct.unpack("b", pkt[report_pkt_offset -2])
							#print "\t(Unknown):", txpower

							rssi, = struct.unpack("b", pkt[report_pkt_offset -1])
							#print "\tRSSI:", rssi
							#distance =  str(10 ** ((abs(rssi)-59)/(10.0 * 2.0)))
							txpower = -59.0
							if deviceBltAddress == "12:3b:6a:1a:62:36" :
								txpower = -49.0
								#print "\t62:36:RSSI:", rssi
							elif deviceBltAddress == "12:3b:6a:1a:5f:4f":
								txpower = -59.0
								print "\t62:36:RSSI:", rssi
							elif deviceBltAddress == "12:3b:6a:1a:5f:3a":
								txpower = -59.0
								print "\t62:36:RSSI:", rssi

							elif deviceBltAddress == "12:3b:6a:1a:62:38":
								txpower = -59.0
								print "\t62:38:RSSI:", rssi
							if txpower > 0 :
								txpower = -70.0
							#ratio = rssi * 1.0 / (-59.0)
							try:
								ratio = rssi * 1.0 / (txpower)
							except:
								return -1
							#distance =  str((0.89976) * pow(ratio,7.7095) + 0.111)
							distance = str((0.42093) * pow(ratio, 6.9476) + 0.54992)
							#distance = pow(10, ((abs(rssi)-(abs(txpower)))/(10*1)))
							#if distance < 0.01:
							#	print "\tDistance: " + "0"
							#else:
							#	print "\tDistance: ", distance
								
							distanceDict[deviceBltAddress] = distance


					# build the return string
					Adstring = packed_bdaddr_to_string(pkt[report_pkt_offset + 3:report_pkt_offset + 9])
					Adstring += ","
					Adstring += returnstringpacket(pkt[report_pkt_offset -22: report_pkt_offset - 6]) 
					Adstring += ","
					Adstring += "%i" % returnnumberpacket(pkt[report_pkt_offset -6: report_pkt_offset - 4]) 
					Adstring += ","
					Adstring += "%i" % returnnumberpacket(pkt[report_pkt_offset -4: report_pkt_offset - 2]) 
					Adstring += ","
					Adstring += "%i" % struct.unpack("b", pkt[report_pkt_offset -2])
					Adstring += ","
					Adstring += "%i" % struct.unpack("b", pkt[report_pkt_offset -1])

					#print "\tAdstring=", Adstring
					myFullList.append(Adstring)
				done = True
	sock.setsockopt( bluez.SOL_HCI, bluez.HCI_FILTER, old_filter )
	#print distanceDict
	return distanceDict