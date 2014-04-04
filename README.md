Master Of Plaster Project Repository

Project Code for working with the IRB 6640 to Plaster Walls.

To Load IK solutions at runtime for the IRB 6640 instead of generating them, run the following command:

```
openrave.py --database inversekinematics --robot=irb6640.KinBody.xml --manipname=plaster_tool  --iktype=Transform6D
```
