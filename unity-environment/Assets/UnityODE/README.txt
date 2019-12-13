The (somewhat) stable API includes the following basic components:


ODEManager.cs: every scene with ODE physics needs this attached to some game object

ODEPhysicsObject.cs: add this to GameObjects with capsule or box colliders to make ODEManager instantiate the corresponding ODE geoms and bodies

MecanimODERig.cs: add this to a mecanim character to create a full character rig controllable with target velocities or target poses. See the MecanimODERigTest scene, where the rig can be posed (driven towards desired joint angles). Note that currently, updating many of the parameters does not have an effect during runtime - they're only applied when Unity calls Start() and the rig is built.





