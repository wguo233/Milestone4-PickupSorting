# Milestone4-PickupSorting
Picking and Sorting mechanism for the MME 4487 Robot Scavenger Project  

The pickup and sorting system is comprised of an arm and a gripper, both controlled using an RC servo motor. The colour detector is placed inside the gripper and will read values when the gripper is closed. The system is initiated using a push button to prevent unintended actions when objects are approaching the colour sensor, but not in the desired place for pickup.  

The process:  
1. On startup, the gripper will close and the colour detector will read RGB values and sum them. This will be the value of ambient light.Once finished, the gripper will open back up. This happens once.  
2. When pushing the button, the gripper will close. Objects will be encapsulated if any are present.
3. When the gripper is in the closed position, the colour detector will read R,G,B,C values.
4. If the resulting sum of the RGB values is similar to the ambient light recorded, it assumes there is no object, and the gripper will open again.  
5. If there is a difference in the recorded summation of RGB values and the ambient light, it assumes there is an object.
6. The white object (undesired) has a high scan value (the summation of RGB) compared to the clear green object (desired), causing the arm and gripper to act differently.
7. For the desired object, the arm will rotate backwards towards the chassis and drop off the object, then rotate back to starting position.
8. For the undesired object, the arm will rotate 90 degrees upwards, then return to starting position and let go of the object.

For demonstration of the Pickup/Sorting system, the button was included in the same file using only one microcontroller.  
For the final robot, the button will be placed on the controller and be included in the controller.ino code from Milestone 3. This pickup and sorting system will also be included in the same file as the drive.ino code similarily.
