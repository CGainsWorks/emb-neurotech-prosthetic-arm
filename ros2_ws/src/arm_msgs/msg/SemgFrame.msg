# One conditioned, decimated sEMG frame as it leaves the board, at 200 Hz.
#
# Eight channels, not the sixteen the classifier was trained on. That is a real
# mismatch and docs/INTEGRATION.md says what has to happen about it.

std_msgs/Header header

# The board's own frame counter. Gaps in it are lost frames on the link, which
# is a different fault to a board that has stopped producing them.
uint32 sequence

# Microseconds from the board's clock, at the instant the last raw sample in
# this frame was taken. Not the host clock: the two are unsynchronised and only
# the board's is uniform.
uint32 device_timestamp_us

int16[8] samples
uint16[8] envelopes

# One bit per channel, set when that channel spent the frame against a rail.
# Any bit set makes the frame unusable for classification.
uint8 saturated_mask
