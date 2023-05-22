
var imports = new JavaImporter(java.io, java.lang, java.util, java.awt);

with (imports) {
    function configure3D() {
        var cc = boxColor.trim().toLowerCase();

        if (!cc.startsWith('0x')) {
            cc = '0x' + cc;
        }
        var filename = 'generated/tagstation_' + instance.uuid + '.ptx';
        var app = parametric.appearance()
                .diffuseColor(Integer.decode(cc))
                .transparency(transparency);

        builder.parametric(filename, box(length, width, height).appearance(app))
                .frame('inFrame', length * 0.5, width * 0.5, height * 0.5, 0, 0, 0);


    };
}