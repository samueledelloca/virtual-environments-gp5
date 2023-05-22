
function configure3D() {
    var filename = 'generated/Encoder_' + instance.uuid + '/body.ptx';
//    var filenameLcd = 'generated/Encoder_' + instance.uuid + '/lcd.ptx';

    var app = parametric.appearance()
            .diffuseColor(0xcc33ff)
            .ambientColor(0xcc33ff)
            .transparency(0.0);
    
//    var appLCD = parametric.appearance()
//            .diffuseColor(0x9BA497)
//            .ambientColor(0x9BA497)
//            .transparency(0.0);

    builder.createChild('pl_geom').parametric(filename, box(50, 100, 100).appearance(app));
//    builder.createChild('lcd').parametric(filenameLcd, box(20, 4, 50).appearance(appLCD)).translate(-10,-109, 150);
    

}
