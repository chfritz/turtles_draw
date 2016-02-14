// Globals

// size of the canvas, will be set when rendering the canvas
var SIZE;
renderer = null;

// shape to draw, will be filled when user clicks
var shape = [];
// the pixi drawing stage
var stage;

var last = null;

// -------------------------------------------------------------------

/** translate from turtlesim coordinates to pixi coordinates */
function translate(pose) {
  var scale = SIZE / 10; // 10 is the bound of turtlesim
  var x = pose.x * scale;
  var y = SIZE - (pose.y * scale);
  pose.x = x;
  pose.y = y;
}

/** inverse of translate */
function translate_inv(coords) {
  console.log(coords);
  var scale = SIZE / 10; // 10 is the bound of turtlesim
  var x = coords.x / scale;
  var y = (SIZE - coords.y) / scale;
  coords.x = x;
  coords.y = y;
}

// -------------------------------------------------------------------

Template.canvas.onRendered( function() {
    Meteor.subscribe('trace');
});

Template.canvas.onRendered( function() {

    // setup pixi
    SIZE = Math.min(window.innerWidth, window.innerHeight);
    renderer = PIXI.autoDetectRenderer(
      SIZE, SIZE, { antialias: true, autoResize: true });
    $("#canvas").get(0).appendChild(renderer.view);

    stage = new PIXI.Container();
    stage.interactive = true;
    renderer.view.onclick = onClick;

    // keep getting traces and draw them on updates
    Trace.find().observe({
        changed: update,
        added: update
      });

    animate();
});

// -------------------------------------------------------------------

// handle mouse clicks: add to shape array; when closed, call the server with it
function onClick(mouseData) {

   var coords = {
    // normalize coordinates in terms of window size (after resizing)
     x: mouseData.x * (SIZE / renderer.view.clientWidth),
     y: mouseData.y * (SIZE / renderer.view.clientHeight)
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
     // teleport turtle to the start
     Meteor.call('teleport', shape[0], function(err, res) {
         if (err) {
           console.log("not drawing, couldn't teleport: ", err);
         } else {
           // clear the canvas
           last = null;
           stage.removeChildren();
           // then order the turtle to draw this
           Meteor.call('draw', shape);
           shape = [];
         }
       });

   } else {

     shape.push(coords);
     var g = new PIXI.Graphics();
     g.lineStyle(0);
     g.beginFill(0xFFFF0B, 0.5);
     g.drawCircle(mouseData.x * (SIZE / renderer.view.clientWidth),
       mouseData.y * (SIZE / renderer.view.clientHeight), 10);
     g.endFill();
     stage.addChild(g);
   }
}

// handling pose updates from server (via collection): draw consecutive polygon
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

// animation loop
function animate() {
  renderer.render(stage);
  requestAnimationFrame( animate );
};


// resize the canvas with the window
window.onresize = function() {
  var newsize = Math.min(window.innerWidth, window.innerHeight);
  $('canvas').get(0).style.width = newsize + "px";
  $('canvas').get(0).style.height = newsize + "px";
}
