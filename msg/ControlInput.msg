# Name of the Control Channel being addressed
# -Users can set a unique name to add
#  additional control inputs to the system
string channel

# Heading is a 2 dimensional vector representing
# the direction the User wants the rover to go
float32[2] heading

# Speed_clamp is a variable that allows Users to
# limit the max speed of the rover. This calculation
# is handled by the node that communicates directly
# with the motor controllers.
float32 speed_clamp

# Is_urgent allows a User to specify a dire need for
# their control input to be obeyed instantly.
# A potential use case is an emergency signal to
# prevent the rover from driving off a cliff.
# Future functionality will allow handling of multiple
# urgent signals through a priority system.
# Currently, the urgent signal being listened to
# is randomly selected through python's dictionary
# construction algorithm.
bool is_urgent