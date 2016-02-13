// Globals

var SIZE;
renderer = null;
var shape = [];

// -------------------------------------------------------------------

/** translate from turtlesim coordinates to pixi coordinates */
function translate(pose) {
  var scale = SIZE / 10; // 10 is the bound of turtlesim
  var x = pose.x * scale;
  var y = SIZE - (pose.y * scale);
  pose.x = x;
  pose.y = y;
}

function translate_inv(coords) {
  console.log(coords);
  var scale = SIZE / 10; // 10 is the bound of turtlesim
  var x = coords.x / scale;
  var y = (SIZE - coords.y) / scale;
  coords.x = x;
  coords.y = y;
}

// -------------------------------------------------------------------

Template.canvas.helpers({
  });

Template.canvas.events({
    // 'click': function(event) {
    //   console.log("event", event);
    // we could also grab the event here, but here pixi doesn't add its sugar
    // }
  });

Template.canvas.onRendered( function(){
    Meteor.subscribe('trace');
});

Template.canvas.onRendered( function(){

    SIZE = Math.min(window.innerWidth, window.innerHeight);
    renderer = PIXI.autoDetectRenderer(
      SIZE, SIZE, { antialias: true, autoResize: true });
    $("#canvas").get(0).appendChild(renderer.view);

    // create the root of the scene graph
    var stage = new PIXI.Container();
    stage.interactive = true;

    renderer.view.onclick = function(mouseData){
       var coords = {
         x: mouseData.x, // / renderer.view.clientWidth,
         y: mouseData.y, // / renderer.view.clientHeight
       }
       translate_inv(coords);
       console.log(coords);

       var distance;
       if (shape.length > 1) {
         diff = {
           x: coords.x - shape[0].x,
           y: coords.y - shape[0].y
         };
         distance = Math.sqrt( diff.x * diff.x + diff.y * diff.y );
       }
       if (distance && distance < 0.1) {
         // the user clicked very close to the origin, close this shape
         shape.push(shape[0]);

         // order the turtle to draw this
         Meteor.call('draw', shape);

         shape = [];
       } else {

         shape.push(coords);
         var g = new PIXI.Graphics();
         g.lineStyle(0);
         g.beginFill(0xFFFF0B, 0.5);
         g.drawCircle(mouseData.x, mouseData.y, 10);
         g.endFill();
         stage.addChild(g);
       }
    }

    // keep getting traces and draw them on updates
    var last = null;
    function update(doc) {
      translate(doc);
      console.log("changed", doc);

      if (last) {
        var g = new PIXI.Graphics();
        g.lineStyle(4, 0xffd900, 1);
        g.moveTo(last.x, last.y);
        g.lineTo(doc.x, doc.y);
        stage.addChild(g);
      }

      last = doc;
    }

    Trace.find().observe({
        changed: update,
        added: update
      });

    // start animation
    function animate() {
      renderer.render(stage);
      requestAnimationFrame( animate );
    };

    animate();
});

// resize the canvas with the window
window.onresize = function() {
  $('canvas').get(0).style.width =
  Math.min( window.innerWidth, window.innerHeight) + "px";
}
