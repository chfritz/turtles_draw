
var ros;
var pose = {};
var pathTopic;

Meteor.startup(function () {

    Trace.remove({});

    ros = new ROSLIB.Ros({
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
        pose = msg;
      });

    // periodically updating the pose communicated to clients
    Meteor.setInterval(function() {
        Trace.upsert("1", pose);
      }, 100);

    pathTopic = new ROSLIB.Topic({
        ros : ros,
        name : '/draw/path',
        messageType : 'nav_msgs/Path'
      });
    pathTopic.advertise();

  });

// make a synchronous call to a ROS service, provide name, type, and data
callServiceSync = Meteor.wrapAsync(function(options, callback) {
  var service = new ROSLIB.Service({
      ros : ros,
      name : options.name,
      serviceType : options.type
    });

  var request = new ROSLIB.ServiceRequest(options.data);

  service.callService(request, function(result) {
      callback(null, result);
    });
});

Meteor.methods({
    'draw': function(shape) {
      // shape is an array of x, y coordinates in the turtlesim coordinate system
      console.log("draw", shape);

      var msg = new ROSLIB.Message({
          poses: _.map(shape, function(point) {
              return {
                pose: {position: point}
              };
            }),
          header: { seq: 0, stamp: 0, frame_id: "world" }
        });
      pathTopic.publish(msg);
    },

    // not yet used:
    // 'spawn': function(x, y) {
    //   callServiceSync({
    //       name: 'spawn',
    //       type: 'turtlesim/Spawn',
    //       data: {x: x, y: y, theta: 0}
    //     });
    // },

    'teleport': function(point) {
      callServiceSync({
          name: 'turtle1/teleport_absolute',
          type: 'turtlesim/TeleportAbsolute',
          data: {x: point.x, y: point.y, theta: 0}
        });

      // clear the drawing area
      callServiceSync({
          name: 'clear',
          type: 'std_srvs/Empty'
        });
    }

  });
