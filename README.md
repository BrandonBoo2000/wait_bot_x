# Wait-Bot-X
Group Project for WID3005 Intelligent Robotic

## Ordering System
This part is an interaction between Wait-Bot-X with customer
First, to initiate the ordering system, you should open terminal and run the *roscore*
```
roscore
```
Next, open another terminal to run below command
```
rosrun wait_bot_x ordering_system.py
```
There are several speech can be detected to perform respective task when ordering is started
- menu: display the menu
- repeat: repeat the order
- done: confirm the order with customer
- thank you: end the ordering system
- food ordering
  - must state the item and the amount of the items
  - example: I want a fried chicken, six burger and etc

Menu
```
menu = ["nasi lemak", "pizza", "burger", "fried chicken", "pepsi", "orange juice", "apple juice", "coffee"]
```
