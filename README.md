If the ".ino" code doesn't work for you and doesn't make the robot balance itself, it is probably because of the KP, KI, and KD parameters. You should adjust these to suit your own setup, as changes in the weight of the setup can affect the robot's stability. Since we are not using the same setup, we will need different parameters. Here are a few tips to keep in mind:
-Use small values for the KD parameter and keep it smaller than the other parameters. This is because KD has a significant impact on the robot and can cause excessive oscillation if the values are too large.
-Use a large value for the KP parameter, something like "112". This is the main parameter responsible for eliminating error and maintaining the robot's balance.
-Adjust the KI parameter to avoid steady-state error.
