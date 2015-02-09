#!/usr/bin/bash

serial_send() {
	rostopic pub -1 "/serial_outgoing_messages" std_msgs/String -- "'$1'"
}

serial_rcv() {
	rostopic echo "/serial_incoming_messages"
}

fabric_send() {
	rostopic pub -1 "/fabric_localization_request" std_msgs/Bool -- True
}

robot_send() {
	rostopic pub -1 "/serial_incoming_messages" std_msgs/String -- "REACHED"
}