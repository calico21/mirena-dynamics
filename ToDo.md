# Pending tasks
## Short-Term
| Number | Name | Explicación | Dependencia | Following tasks | Responsible | Due Date | To check | Done | 
| --- | --------------- | ---------- | ------------ | ------------| --------- | ------ | --- | --- | 
| 1 | Revise rules of support | See if LiDar support is compliant with rules (Pilot helmet hits support) | Not dependant | If it is not compliant (or if there is a slight possibility of it not being accepted) think how to hold it differently or how to change the structure| | | | |
| 2 | Remote control | Code in Godot the remote control interface for the car, it should take the camera feed an display it as a texture and pass the controller input as control | Not dependant | None | | | | |
| 3 | Bring Up (concepto)| Decide how we want the Bring up structure to be, think about the best common topics to have and the different modes and remmapings needed | Not dependant | After deciding structure we need to finish the node and make it work seamlessly | | | | |
| 4 | Sensor Cover | Design and 3D print a cover to protect the sensors, it should be asey to place and to remove (Probably the best is to have a 2 piece system that covers from the top) | Task 1 | None | | | | |
| 5 | Finetune Planning | Planning should be robust even if slam fails slighty, we must think how we want to treat the case of the 4 orange cones (do we want it to have a concrete method to to it?)| For this to work IMU and Slam must be validated | None | | | | |
| 6 | Integrate CanBus in Hardware | CanBus functionality should be part of Hardware | We must first validate CanBus | None | | | | |
| 7 | Update Brakes Diagram | New structure with changes| Not Dependant | None | Cheng | 15/06 | | |

## Long Term
| Number | Name | Explicación | 
| --- | --------------- | ---------- | 
| 1 | Sistema iteracion continua | Create a series of checks and tests every branch must pass in order to be able to merge into the main branch (Run a couple of laps in the sim, topic's data flow is correct ...)|
| 2 | Implement pull requests | limit commit capabilities and make it mandatory to use a pull request to integrate changes|

## Think about
### If you have nothing to do and you are bored this is a list of things you can start looking at

* AI based perception
* 


