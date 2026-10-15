# One decision from the gesture classifier.
#
# The class codes are Ninapro DB5 exercise B, which is what
# ml-semg-gesture-classifier is trained on: 0 is rest and 1 to 17 are the
# movements in that exercise, in the dataset's own order. docs/INTEGRATION.md
# has the table and the mapping onto arm poses.

std_msgs/Header header

uint8 REST = 0

uint8 gesture_id
string gesture_name

# Softmax probability of the reported class. The mapper refuses to act below
# a configured threshold: a wrong grasp is worse than no grasp.
float32 confidence

# False when the classifier could not produce a decision at all, which is
# different from a low-confidence one. An electrode that has lifted produces
# this rather than a confident guess about rubbish.
bool valid

# Milliseconds between the end of the window this decision came from and the
# moment it was published. The window itself is 200 ms and is not counted here;
# docs/CONTROL_LOOP_TIMING.md adds the two up.
float32 inference_latency_ms
