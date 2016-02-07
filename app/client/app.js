Session.setDefault('counter', 0);

Template.canvas.helpers({
  });

Template.canvas.events({
  });

renderer = null;
Template.canvas.onRendered( function(){

    var SIZE = Math.min(window.innerWidth, window.innerHeight);
    renderer = PIXI.autoDetectRenderer(
      SIZE, SIZE, { antialias: true });
    $("#canvas").get(0).appendChild(renderer.view);

    // create the root of the scene graph
    var stage = new PIXI.Container();
    stage.interactive = true;

    function animate() {
      renderer.render(stage);
      requestAnimationFrame( animate );
    };

    Meteor.subscribe('trace');

    var last = null;

    function transpose(pose) {
      var SCALE = SIZE / 10; // 10 is the bounds of turtlesim
      var x = pose.x * SCALE;
      var y = SIZE - (pose.y * SCALE);
      pose.x = x;
      pose.y = y;
    }

    function update(id, fields) {
      transpose(fields);
      console.log("changed", fields);
      if (last) {
        var g = new PIXI.Graphics();
        g.lineStyle(4, 0xffd900, 1);
        g.moveTo(last.x, last.y);
        g.lineTo(fields.x, fields.y);
        stage.addChild(g);
      }

      last = fields;
    }

    Trace.find().observeChanges({
        changed: update,
        added: update
      });

    // start animation
    animate();
});


window.onresize = function() {
  $('canvas').get(0).style.width =
  Math.min( window.innerWidth, window.innerHeight) + "px";
}
