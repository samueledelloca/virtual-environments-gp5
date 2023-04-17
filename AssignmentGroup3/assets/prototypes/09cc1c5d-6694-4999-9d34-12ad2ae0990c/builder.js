
var imports = new JavaImporter(java.io, java.lang, java.util, java.awt);

with (imports) {
    function configure3D() {

        var filename = 'generated/screwdriver_' + instance.uuid + '.ptx';
        var fnConn = 'generated/screwdriverconn_' + instance.uuid + '.ptx';
        var app = parametric.appearance()
//                .specularColor(Integer.decode('0x0b3963'))
                .diffuseColor(Integer.decode('0xaacff2'));

        var geom = revolution()
                .line(0, 45)
                .arc(15, 60, 15, true)
                .line(220, 60)
                .arc(20, 30, 80, true)
                .line(0, 25)
                .line(70, 25)
                .line(20, 10)
                .appearance(app);
        var connector = revolution()
                .line(0, 40)
                .line(70, 40)
                .line(0, 60)
                .line(5, 60)
                .appearance(app);

        var h = builder.createChild('holder').rotate(0, 180, 0).translate(0,0,345);
        
        builder.frame('attach',0,-95,245,90,0,90);
        builder.frame('tcp',0,0,0,0,0,90);

        var conn = h.createChild('connector').parametric(fnConn, connector)
                .frame('inFrame', length * 0.5, width * 0.5, height * 0.5, 0, 0, 0);
        conn.rotate(90, 0, 0).translate(0, -20, 100);

        var sd = h.createChild('driver').parametric(filename, geom)
                .frame('inFrame', length * 0.5, width * 0.5, height * 0.5, 0, 0, 0);




    };
}