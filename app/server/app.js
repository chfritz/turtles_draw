Meteor.startup(function () {
    // code to run on server at startup

    var ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
    });

    ros.on('connection', function() {
    console.log('Connected to websocket server.');
    });

    ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function() {
    console.log('Connection to websocket server closed.');
    });

    var topic_pose = new ROSLIB.Topic({
      ros : ros,
      name : '/turtle1/pose',
      messageType : 'turtlesim/Pose'
    });

    topic_pose.subscribe(function(msg) {
        console.log("got message: ", msg);
      });

  });
