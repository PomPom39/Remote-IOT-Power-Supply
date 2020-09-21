###Remote IOT Power Supply
This project aims at giving remote power on/off capabilites(via LAN) to an already pre-existing regulated DC power supply. 

##What is the Purpose of this?
Among this Covid pandemic, many companies have resorted to Work From Home policies. Although its going well for purely software based companies, hardware-based embedded companies, whose activites rely on debugging, testing, development etc, have taken a toll. Employees can do 90% of the work right from the comfort of their home, but sometimes they may get into situations where a power-on reset to a board/system is required. The goal is to create a low-cost attachable module consiting of a microcontroller, ethernet PHY and a simple GUI to which acts as a intermediate between the power supply and device

##Dont we already have ethernet-controlled power supplies? Why not get those?
Yes we do have plenty of them, but this project tries to incorporate this feature to a already existing old power DC regulated power supply

##What have you used?
1) TI MSP432P401R LaunchPad Kit(for development)
2) ENC28J60 Ethernet-Controller with SPI 
3) A Relay
4) Regulated Power Supply

##What is the status of the project?
Still working on it