# dtn_proxy

This package contains a ROS-DTN proxy which enables ROS communication over DTN.  
Requires a running `dtnd` instance of the [dtn7-rs](https://github.com/dtn7/dtn7-rs) project.

Contents:

- [config/node0.toml](config/node0.toml): Example Configuration file
- [external/](external/): launch files
- [launch/](launch/): launch file

## Configuration

Edit the configuration file in [config/node0.toml](config/node0.toml).  
Mandatory parameters are listed under the `[dtn]` tag.

The following modules are currently available for use in optimization profiles:  

| Module    | Parameter                  | Description                                                   |
|-----------|----------------------------|---------------------------------------------------------------|
| Combine   | topic1, topic2, ... topicN | Combines messages from multiple topics before transmission    |
| CombineTF | tf_frame1, tf_frame2       | Appends a `tf2` transform message from tf_frame1 to tf_frame2 |
| Compress  | -                          | Compresses messages before transmission                       |
| Expire    | -                          | Marks old messages as expired                                 |
| OnChange  | -                          | Only sends messages that have changed from the previous one   |
| RateLimit | Duration in seconds        | Limits the topic rate to one message per time duration        |

> All parameters are expected as string values.

## Usage

Launch the proxy using `ros2 launch dtn_proxy default_launch.py`.  
Optional launch parameters can change the config file (`configurationPath`) and the log level (`log_level`).

> Example:  
> `ros2 launch dtn_proxy default_launch.py log_level:=INFO configurationPath:=path/to/config.toml`
