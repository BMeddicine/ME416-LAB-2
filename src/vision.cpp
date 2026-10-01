// Vision: camera frame -> table coordinates [U, V].
// - Detect AprilTags (tag36h11) and get each tag's pixel center and corners
// - Build H (table plane -> pixel) from the base tag
// - Build H_h for raised planes (stamp discs), or decide to accept the parallax
// - Convert a tag's pixel center to [U, V] and its yaw on the table
// - Apply an offset given in the tag's own frame (e.g. tag -> steel disc, Problem 1(a))
