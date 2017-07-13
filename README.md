A simple Qt panel that displays messages published on a ROS topic. The text can
be styled with CSS.

The point of this package is to provide an easy way to show users information,
without forcing them to read scrolling lines of terminal output.

To publish to the status panel, publish `std_msgs/String` messages to the
`/status` topic. This topic can be changed, but `\status` is the default value.