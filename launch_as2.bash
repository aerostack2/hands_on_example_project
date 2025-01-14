#!/bin/bash

usage() {
    echo "  options:"
    echo "      -s: simulated, choices: [true | false]"
    echo "      -e: estimator_type, choices: [ground_truth, raw_odometry, mocap_pose]"
    echo "      -t: launch keyboard teleoperation"
}

# Arg parser
while getopts "se:mrtv" opt; do
    case ${opt} in
        s )
            simulated="true"
            ;;
        e )
            estimator_plugin="${OPTARG}"
            ;;
        t )
            launch_keyboard_teleop="true"
            ;;
        \? )
            echo "Invalid option: -$OPTARG" >&2
            usage
            exit 1
            ;;
        : )
            if [[ ! $OPTARG =~ ^[swrt]$ ]]; then
                echo "Option -$OPTARG requires an argument" >&2
                usage
                exit 1
            fi
            ;;
    esac
done

# Shift optional args
shift $((OPTIND -1))

## DEFAULTS
estimator_plugin=${estimator_plugin:="raw_odometry"}
simulated=${simulated:="false"}
# If simulated, estimator_plugin is ground_truth
if [[ ${simulated} == "true" ]]; then
    estimator_plugin="ground_truth"
fi
rviz=${rviz:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}

simulation_config="real_config/swarm_config_file.yaml"
if [[ ${simulated} == "true" ]]; then
    simulation_config="sim_config/world.json"
fi

# Fron swarm config, get the list of drone namespaces
drone_namespaces=$(python3 utils/get_drones.py ${simulation_config})
drone_namespaces_list=($(echo $drone_namespaces | tr ':' ' '))

for ns in "${drone_namespaces_list[@]}"
do
    base_launch="false"
    if [[ ${ns} == ${drone_namespaces_list[0]} ]]; then
        base_launch="true"
    fi 

    tmuxinator start -n ${ns} -p utils/aerostack2.yml \
        drone_namespace=${ns} \
        base_launch=${base_launch} \
        estimator_plugin=${estimator_plugin} \
        simulation=${simulated} \
        simulation_config=${simulation_config} &
    wait
done

if [[ ${estimator_plugin} == "mocap_pose" ]]; then
    tmuxinator start -n mocap -p utils/mocap4ros2.yml &
    wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
    drone_namespaces_comma=$(echo $drone_namespaces | tr ':' ',')
    tmuxinator start -n keyboard_teleop -p utils/keyboard_teleop.yml \
        simulation=${simulated} \
        drone_namespaces=${drone_namespaces_comma} &
    wait
fi

if [[ ${simulated} == "true" ]]; then
    tmuxinator start -n gazebo -p utils/gazebo.yml \
        simulation_config=${simulation_config} &
    wait
fi

# Attach to tmux session ${drone_ns[@]}, window 0
tmux attach-session -t ${drone_namespaces_list[0]}:mission
