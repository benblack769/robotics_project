DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
#echo $DIR/models/
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$DIR/models/
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$DIR/../../devel/lib/
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$DIR/world/
