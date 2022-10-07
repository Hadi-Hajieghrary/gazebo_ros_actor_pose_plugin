# gazebo_ros_actor_pose_plugin
A plugin to publish the pose of the actor(s) inside the gazebo simulation on a ros topic.

The plugin should be added under each of the "actor" tags in the word file:

      <plugin name="actor1_plugin" filename="libgazebo_ros_actor_pose.so">
        <topicName>actor1_pose</topicName>
        <updateRate>10</updateRate>
      </plugin>
      
In the topic is not specified, the topic name is going to be determined as <actor name>_pose.

To see the example run:

roslaunch gazebo_worlds load_warehouse.launch

