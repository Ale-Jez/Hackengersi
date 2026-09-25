Driving trash can

Navigating via hardcoded routes and april tags



then there are two robo arms, 1xRoArm which is the main one for picking up trash
and 1xSo-arm-101 which has the camera and takes a picture of the trash can content.

both of them and the camera are managed by the raspberry pi 5 8gb. 
camera and so-101 are connected via usb cable to the raspberry
and the roarm m3 is connected via wifi to the raspberry pi

the raspberry pi manages these two arms and sends the images of the trash can to my nvidia brev, where there will be an LLM analyzing this photo and what there is, then it should map the objects so that the arms can pick the trash and sort them to one of the three containers.

One container: cans and bottles
second container: paper
third container: random plastic



