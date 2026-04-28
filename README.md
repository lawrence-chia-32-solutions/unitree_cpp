# Unitree Cpp

A lightweight Python binding for **Unitree SDK2**, designed to overcome the performance issues of the official `unitree_sdk2_python` on the **Unitree G1** (Jetson Orin).  

**FREE YOUR UNITREE G1 FROM THE ETHERNET CABLE!**

<div align="center">
<h3><strong>News 🎉: Our Deployment Framework is released at <a href="https://github.com/GDDG08/RoboJuDo">RoboJuDo</a>, try it out!</strong></h3>
<br>
</div>

<div align="center">
<img src="asset/image.png" width="50%"/>
</div>

## Inspiration

On Unitree G1, `unitree_sdk2_python` often suffers from serious performance issues, making it difficult to achieve real-time deploy with built-in Jetson pc2. 

Using `unitree_sdk2` directly avoids these performance problems, but C++ development and compilation can be cumbersome and time-consuming. 

This project provides the best of both worlds:  
- **Python interface** for simplicity and quick prototyping  
- **C++ backend** for high-frequency communication and efficiency  

As a result, you can write simple control code in Python, without dealing with the C++ compilation, while still ensuring real-time performance.


We tested the performance of `unitree_cpp` with [AMO](https://github.com/OpenTeleVision/AMO) on G1 pc2. The results are as follows:

<div align="center">

| Implementation      | Cost of control (μs/hit) |
|---------------------|---------------|
| `unitree_sdk2_python`  | 3871.039      |
| `unitree_cpp`          | 40.045        |

</div>

## Installation

### 1. Install Unitree SDK2

Follow the instructions on [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)  
to install the SDK2 on your system.  

**Note:** It is recommended to use the default installation path.  
If you choose a custom path, make sure to update `CMakeLists.txt` accordingly,  
so that the required libraries can be found.


### 2. Install `unitree_cpp` Python Binding

```bash
# switch to your python env
# tested on python>=3.8
pip install .
```

## Getting Started
Please refer to the example file: [`example/unitree_cpp_env.py`](example/unitree_cpp_env.py). 

Test the example with your robot:
- `pip install -r example/requirements.txt`
- change ethernet interface `eth_if` in [`example/config.py`](example/config.py)
- run the example
- your robot should move slowly into default position

This project supports:  
- **Unitree G1**  
- **Dex-3 hand**  
- **Odometry service**  

## LowCmd Trace Logging (for jerk diagnosis)

`UnitreeController` can log every published low-level command (`rt/lowcmd`) and hand command write in JSONL format.

### Enable

```bash
export UNITREE_CPP_TRACE_ENABLE=1
export UNITREE_CPP_TRACE_FILE=/tmp/unitree_cpp_lowcmd_trace.jsonl
export UNITREE_CPP_TRACE_FLUSH=1
export UNITREE_CPP_TRACE_SAMPLE=1
```

### Environment variables

- `UNITREE_CPP_TRACE_ENABLE`: `1`/`0` (default `1`)
- `UNITREE_CPP_TRACE_FILE`: output file path (default `/tmp/unitree_cpp_lowcmd_trace.jsonl`)
- `UNITREE_CPP_TRACE_FLUSH`: flush after each line (`1` default)
- `UNITREE_CPP_TRACE_SAMPLE`: log every `N`th command (`1` default means all commands)

### Recorded fields

- `stream`: `lowcmd` or `handcmd`
- `seq`: per-stream sequence number
- `mono_us`: monotonic timestamp in microseconds
- `publish_dt_us`: interval from previous publish on that stream
- For `lowcmd`: `control_dt_us`, `dt_out_of_band`, `lowstate_tick`, `lowstate_tick_dt`, `mode_pr`, `mode_machine`, `control_mode`, `crc`, full `motors[]` payload (`mode`, `q`, `dq`, `tau`, `kp`, `kd`)
- For `handcmd`: `side`, full `motors[]` payload

### Quick checks

```bash
# First lines
head -n 5 /tmp/unitree_cpp_lowcmd_trace.jsonl

# Distribution of publish period for lowcmd
jq -r 'select(.stream=="lowcmd") | .publish_dt_us' /tmp/unitree_cpp_lowcmd_trace.jsonl | sort -n | uniq -c

# Commands where timing drifted far from configured control_dt
jq -c 'select(.stream=="lowcmd" and .dt_out_of_band==true)' /tmp/unitree_cpp_lowcmd_trace.jsonl | head
```

## CHANGELOG

**1.0.2**
- Fix: shutdown as damping mode

**1.0.3 [IMPORTANT FIX]**
- Fix control delay, send command immediately after step.
    - this bug could lead to jittering and stability issue, see https://github.com/GDDG08/RoboJuDo/issues/2


## License

CC-BY-4.0
