
var pose = {};

Meteor.startup(function () {

    Trace.remove({});

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
        // Trace.insert(msg);
        // ^^ no can do that! not in a fiber, plus this would be a little
        // verbose
        pose = msg;
      });

    // periodically updating the pose communicated to clients
    Meteor.setInterval(function() {
        Trace.upsert("1", pose);
      }, 200);
    // #TODO: this is a little brittle. If the client misses some of these then
    // there will be gaps in the drawing, including cut corners. So maybe use
    // insert afterall? It's not more bandwidth just more data that is
    // (volatile) and really doesn't need to go into the db.

  });
