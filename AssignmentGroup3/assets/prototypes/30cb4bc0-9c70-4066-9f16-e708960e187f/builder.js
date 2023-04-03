

var height = 800;
var width = 200;
var length = 200;

function configure3D() {
    var modelFile = localFile('Spintore.xml');
    var pusher = builder.createChildModel(modelFile);
    
    
    var filename = 'generated/pusher_' + instance.uuid + '.ptx';
    var app = parametric.appearance()
            .diffuseColor(0xFF00EE)
            .transparency(0.8);
//    
//    builder.parametric(filename, box(6000, maxBatchWidth, hPiano).appearance(app))

    pusher.createChild('pc_geom').parametric(filename, box(length, width, height).appearance(app))
            .translate(0,0,0);
    pusher.frame('infoFrame', 0, 100, -300, 0, 0, 0);
    pusher.rotate(90,0,0);

}
