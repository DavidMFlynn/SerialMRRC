# SerialMRRC
 Serial Model Railroad Control

This is a set of PCBs and software to control a model railroad with RS485 data buss.

# Stand Alone Boards
Controller Board, PIC15F15345, 1KB Ram, 224B Flash, 8KW Program memory
Switch Machine, 1 servo, 1 DPDT relay
Switch Machine and Signals, 1 servo, 1 DPDT relay, 5 Signal heads

# Daughter Cards
USB Daughter Board
Input 32 Daughter Bouad, 32 Pull-to-Ground inputs
Output 32 Daughter Board, 32 Open collector Pull-to-Ground outputs
Lighting 8x Transistot Outputs, +12v resistor current limited outputs

Anamation Controller, 8 servos
Block Detector, detects voltage across diode/resistor
Optical Detector, broken beam or reflected beam

# USB Operation
 When a command packet is received from RX2 (USB):
  Source=Master, Destination=Me: Parse and Process the command.
    Send any requested data back to Master on TX2 (USB).
  Source=Master, Destination<>Me: Retransmit on TX1 (RS485)

 When a command packet is received from RX1 (RS485):
  Source=any, Destination=Me: Parse and Process the command. 
     Send any requested data back to the sender on TX1 (RS485).
  Source=any, Destination=Master: Retransmit on TX2 (USB)
  Source=any, Destination= other than Master or Me do nothing.

# Node (w/o USB)
 When a command packet is received from RX1 (RS485):
  Source=any, Destination=Me: Parse and Process the command. 
     Send any requested data back to the sender on TX1 (RS485).
  Source=any, Destination<>Me do nothing.
