
// ======================================================================
//  Low-level canvas access.
// ======================================================================

var canvas = document.getElementById("canvas");
var canvas_context = canvas.getContext("2d");
var canvas_buffer = canvas_context.getImageData(0, 0, canvas.width, canvas.height);
var canvas_pitch = canvas_buffer.width * 4;


// The PutPixel() function.
var PutPixel = function (x, y, color) {
    x = canvas.width / 2 + (x | 0);
    y = canvas.height / 2 - (y | 0) - 1;

    if (x < 0 || x >= canvas.width || y < 0 || y >= canvas.height) {
        return;
    }

    var offset = 4 * x + canvas_pitch * y;
    canvas_buffer.data[offset++] = color[0];
    canvas_buffer.data[offset++] = color[1];
    canvas_buffer.data[offset++] = color[2];
    canvas_buffer.data[offset++] = 255; // Alpha = 255 (full opacity)
}


// Displays the contents of the offscreen buffer into the canvas.
var UpdateCanvas = function () {
    canvas_context.putImageData(canvas_buffer, 0, 0);
}


// ======================================================================
//  Depth buffer.
// ======================================================================
var depth_buffer = Array();
depth_buffer.length = canvas.width * canvas.height;

var UpdateDepthBufferIfCloser = function (x, y, inv_z) {
    x = canvas.width / 2 + (x | 0);
    y = canvas.height / 2 - (y | 0) - 1;

    if (x < 0 || x >= canvas.width || y < 0 || y >= canvas.height) {
        return false;
    }

    var offset = x + canvas.width * y;
    if (depth_buffer[offset] == undefined || depth_buffer[offset] < inv_z) {
        depth_buffer[offset] = inv_z;
        return true;
    }
    return false;
}

var ClearAll = function () {
    canvas.width = canvas.width;
    depth_buffer = Array();
    depth_buffer.length = canvas.width * canvas.height;
}


// ======================================================================
//  Data model.
// ======================================================================

// A Point.
var Pt = function (x, y, h) {
    if (!(this instanceof Pt)) { return new Pt(x, y, h); }

    this.x = x;
    this.y = y;
    this.h = h;
}


// A 3D vertex.
var Vertex = function (x, y, z) {
    if (!(this instanceof Vertex)) { return new Vertex(x, y, z); }

    this.x = x;
    this.y = y;
    this.z = z;
}


// A 4D vertex (a 3D vertex in homogeneous coordinates).
var Vertex4 = function (arg1, y, z, w) {
    if (!(this instanceof Vertex4)) { return new Vertex4(arg1, y, z, w); }

    if (arg1 instanceof Vertex) {
        this.x = arg1.x;
        this.y = arg1.y;
        this.z = arg1.z;
        this.w = 1;
    } else if (arg1 instanceof Vertex4) {
        this.x = arg1.x;
        this.y = arg1.y;
        this.z = arg1.z;
        this.w = arg1.w;
    } else {
        this.x = arg1;
        this.y = y;
        this.z = z;
        this.w = w;
    }
}


// A 4x4 matrix.
var Mat4x4 = function (data) {
    if (!(this instanceof Mat4x4)) { return new Mat4x4(data); }

    this.data = data;
}


var Identity4x4 = Mat4x4([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]);


// A Triangle.
var Triangle = function (indexes, color) {
    if (!(this instanceof Triangle)) { return new Triangle(indexes, color); }

    this.indexes = indexes;
    this.color = color;
}


// A Model.
var Model = function (vertices, triangles, bounds_center, bounds_radius) {
    if (!(this instanceof Model)) { return new Model(vertices, triangles, bounds_center, bounds_radius); }

    this.vertices = vertices;
    this.triangles = triangles;
    this.bounds_center = bounds_center;
    this.bounds_radius = bounds_radius;
}


// An Instance.
var Instance = function (model, position, orientation, scale) {
    if (!(this instanceof Instance)) { return new Instance(model, position, orientation, scale); }

    this.model = model;
    this.position = position;
    this.orientation = orientation || Identity4x4;
    this.scale = scale || 1.0;

    this.transform = MultiplyMM4(MakeTranslationMatrix(this.position), MultiplyMM4(this.orientation, MakeScalingMatrix(this.scale)));
}


// The Camera.
var Camera = function (position, orientation) {
    if (!(this instanceof Camera)) { return new Camera(position, orientation); }

    this.position = position;
    this.orientation = orientation;
    this.clipping_planes = [];
}


// A Clipping Plane.
var Plane = function (normal, distance) {
    if (!(this instanceof Plane)) { return new Plane(normal, distance); }

    this.normal = normal;
    this.distance = distance;
}


// ======================================================================
//  Linear algebra and helpers.
// ======================================================================

// Computes k * vec.
var Multiply = function (k, vec) {
    return Vertex(k * vec.x, k * vec.y, k * vec.z);
}


// Computes dot product.
var Dot = function (v1, v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}


// Computes cross product.
var Cross = function (v1, v2) {
    return Vertex(
        v1.y * v2.z - v1.z * v2.y,
        v1.z * v2.x - v1.x * v2.z,
        v1.x * v2.y - v1.y * v2.x);
}


// Computes v1 + v2.
var Add = function (v1, v2) {
    return Vertex(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}


// Makes a transform matrix for a rotation around the OY axis.
var MakeOYRotationMatrix = function (degrees) {
    var cos = Math.cos(degrees * Math.PI / 180.0);
    var sin = Math.sin(degrees * Math.PI / 180.0);

    return Mat4x4([[cos, 0, -sin, 0],
    [0, 1, 0, 0],
    [sin, 0, cos, 0],
    [0, 0, 0, 1]])
}


// Makes a transform matrix for a translation.
var MakeTranslationMatrix = function (translation) {
    return Mat4x4([[1, 0, 0, translation.x],
    [0, 1, 0, translation.y],
    [0, 0, 1, translation.z],
    [0, 0, 0, 1]]);
}


// Makes a transform matrix for a scaling.
var MakeScalingMatrix = function (scale) {
    return Mat4x4([[scale, 0, 0, 0],
    [0, scale, 0, 0],
    [0, 0, scale, 0],
    [0, 0, 0, 1]]);
}


// Multiplies a 4x4 matrix and a 4D vector.
var MultiplyMV = function (mat4x4, vec4) {
    var result = [0, 0, 0, 0];
    var vec = [vec4.x, vec4.y, vec4.z, vec4.w];

    for (var i = 0; i < 4; i++) {
        for (var j = 0; j < 4; j++) {
            result[i] += mat4x4.data[i][j] * vec[j];
        }
    }

    return Vertex4(result[0], result[1], result[2], result[3]);
}


// Multiplies two 4x4 matrices.
var MultiplyMM4 = function (matA, matB) {
    var result = Mat4x4([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]);

    for (var i = 0; i < 4; i++) {
        for (var j = 0; j < 4; j++) {
            for (var k = 0; k < 4; k++) {
                result.data[i][j] += matA.data[i][k] * matB.data[k][j];
            }
        }
    }

    return result;
}


// Transposes a 4x4 matrix.
var Transposed = function (mat) {
    var result = Mat4x4([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]);
    for (var i = 0; i < 4; i++) {
        for (var j = 0; j < 4; j++) {
            result.data[i][j] = mat.data[j][i];
        }
    }
    return result;
}


var Shuffle = function (vec) {
    for (var i = vec.length - 1; i > 0; --i) {
        var rand = Math.floor(Math.random() * (i + 1));
        [vec[i], vec[rand]] = [vec[rand], vec[i]];
    }
}


var MultiplyColor = function (k, color) {
    return [k * color[0], k * color[1], k * color[2]];
}

// ======================================================================
//  Rasterization code.
// ======================================================================

// Scene setup.
var viewport_size = 1;
var projection_plane_z = 1;


var Interpolate = function (i0, d0, i1, d1) {
    if (i0 == i1) {
        return [d0];
    }

    var values = [];
    var a = (d1 - d0) / (i1 - i0);
    var d = d0;
    for (var i = i0; i <= i1; i++) {
        values.push(d);
        d += a;
    }

    return values;
}


var DrawLine = function (p0, p1, color) {
    var dx = p1.x - p0.x, dy = p1.y - p0.y;

    if (Math.abs(dx) > Math.abs(dy)) {
        // The line is horizontal-ish. Make sure it's left to right.
        if (dx < 0) { var swap = p0; p0 = p1; p1 = swap; }

        // Compute the Y values and draw.
        var ys = Interpolate(p0.x, p0.y, p1.x, p1.y);
        for (var x = p0.x; x <= p1.x; x++) {
            PutPixel(x, ys[(x - p0.x) | 0], color);
        }
    } else {
        // The line is verical-ish. Make sure it's bottom to top.
        if (dy < 0) { var swap = p0; p0 = p1; p1 = swap; }

        // Compute the X values and draw.
        var xs = Interpolate(p0.y, p0.x, p1.y, p1.x);
        for (var y = p0.y; y <= p1.y; y++) {
            PutPixel(xs[(y - p0.y) | 0], y, color);
        }
    }
}


var DrawWireframeTriangle = function (p0, p1, p2, color) {
    DrawLine(p0, p1, color);
    DrawLine(p1, p2, color);
    DrawLine(p0, p2, color);
}


// Converts 2D viewport coordinates to 2D canvas coordinates.
var ViewportToCanvas = function (p2d) {
    return Pt((p2d.x * canvas.width / viewport_size) | 0,
        (p2d.y * canvas.height / viewport_size) | 0);
}


var ProjectVertex = function (v) {
    return ViewportToCanvas(Pt(v.x * projection_plane_z / v.z,
        v.y * projection_plane_z / v.z));
}


// Sort the points from bottom to top.
// Technically, sort the indexes to the vertex indexes in the triangle from bottom to top.
var SortedVertexIndexes = function (vertex_indexes, projected) {
    indexes = [0, 1, 2];

    if (projected[vertex_indexes[indexes[1]]].y < projected[vertex_indexes[indexes[0]]].y) { var swap = indexes[0]; indexes[0] = indexes[1]; indexes[1] = swap; }
    if (projected[vertex_indexes[indexes[2]]].y < projected[vertex_indexes[indexes[0]]].y) { var swap = indexes[0]; indexes[0] = indexes[2]; indexes[2] = swap; }
    if (projected[vertex_indexes[indexes[2]]].y < projected[vertex_indexes[indexes[1]]].y) { var swap = indexes[1]; indexes[1] = indexes[2]; indexes[2] = swap; }

    return indexes;
}


var ComputeTriangleNormal = function (v0, v1, v2) {
    var v0v1 = Add(v1, Multiply(-1, v0));
    var v0v2 = Add(v2, Multiply(-1, v0));
    return Cross(v0v1, v0v2);
}


var EdgeInterpolate = function (y0, v0, y1, v1, y2, v2) {
    var v01 = Interpolate(y0, v0, y1, v1);
    var v12 = Interpolate(y1, v1, y2, v2);
    var v02 = Interpolate(y0, v0, y2, v2);
    v01.pop();
    var v012 = v01.concat(v12);
    return [v02, v012];
}

// Controls depth buffering and backface culling.
var depthBufferingEnabled = true;
var backfaceCullingEnabled = true;

var drawOutlines = false;

var RenderTriangle = function (triangle, vertices, projected) {
    // Sort by projected point Y.
    var indexes = SortedVertexIndexes(triangle.indexes, projected);
    var [i0, i1, i2] = indexes;

    var v0 = vertices[triangle.indexes[i0]];
    var v1 = vertices[triangle.indexes[i1]];
    var v2 = vertices[triangle.indexes[i2]];

    // Compute triangle normal. Use the unsorted vertices, otherwise the winding of the points may change.
    var normal = ComputeTriangleNormal(vertices[triangle.indexes[0]], vertices[triangle.indexes[1]], vertices[triangle.indexes[2]]);

    // Backface culling.
    if (backfaceCullingEnabled) {
        var vertex = vertices[triangle.indexes[0]];
        if (Dot(vertex, normal) <= 0) {
            return;
        }
    }

    // Get attribute values (X, 1/Z) at the vertices.
    var p0 = projected[triangle.indexes[i0]];
    var p1 = projected[triangle.indexes[i1]];
    var p2 = projected[triangle.indexes[i2]];

    // Compute attribute values at the edges.
    var [x02, x012] = EdgeInterpolate(p0.y, p0.x, p1.y, p1.x, p2.y, p2.x);
    var [iz02, iz012] = EdgeInterpolate(p0.y, 1.0 / v0.z, p1.y, 1.0 / v1.z, p2.y, 1.0 / v2.z);


    // Determine which is left and which is right.
    var m = (x02.length / 2) | 0;
    if (x02[m] < x012[m]) {
        var [x_left, x_right] = [x02, x012];
        var [iz_left, iz_right] = [iz02, iz012];
    } else {
        var [x_left, x_right] = [x012, x02];
        var [iz_left, iz_right] = [iz012, iz02];
    }

    // Draw horizontal segments.
    for (var y = p0.y; y <= p2.y; y++) {
        var [xl, xr] = [x_left[y - p0.y] | 0, x_right[y - p0.y] | 0];

        // Interpolate attributes for this scanline.
        var [zl, zr] = [iz_left[y - p0.y], iz_right[y - p0.y]];
        var zscan = Interpolate(xl, zl, xr, zr);

        for (var x = xl; x <= xr; x++) {
            if (!depthBufferingEnabled || UpdateDepthBufferIfCloser(x, y, zscan[x - xl])) {
                PutPixel(x, y, triangle.color);
            }
        }
    }

    if (drawOutlines) {
        var outline_color = MultiplyColor(0.75, triangle.color);
        DrawLine(p0, p1, outline_color);
        DrawLine(p0, p2, outline_color);
        DrawLine(p2, p1, outline_color);
    }
}


// Clips a triangle against a plane. Adds output to triangles and vertices.
var ClipTriangle = function (triangle, plane, triangles, vertices) {
    var v0 = vertices[triangle.indexes[0]];
    var v1 = vertices[triangle.indexes[1]];
    var v2 = vertices[triangle.indexes[2]];

    var in0 = Dot(plane.normal, v0) + plane.distance > 0;
    var in1 = Dot(plane.normal, v1) + plane.distance > 0;
    var in2 = Dot(plane.normal, v2) + plane.distance > 0;

    var in_count = in0 + in1 + in2;
    if (in_count == 0) {
        // Nothing to do - the triangle is fully clipped out.
    } else if (in_count == 3) {
        // The triangle is fully in front of the plane.
        triangles.push(triangle);
    } else if (in_count == 1) {
        // The triangle has one vertex in. Output is one clipped triangle.
    } else if (in_count == 2) {
        // The triangle has two vertices in. Output is two clipped triangles.
    }
}


var TransformAndClip = function (clipping_planes, model, scale, transform) {
    // Transform the bounding sphere, and attempt early discard.
    center = MultiplyMV(transform, Vertex4(model.bounds_center));
    var radius = model.bounds_radius * scale;
    for (var p = 0; p < clipping_planes.length; p++) {
        var distance = Dot(clipping_planes[p].normal, center) + clipping_planes[p].distance;
        if (distance < -radius) {
            return null;
        }
    }

    // Apply modelview transform.
    var vertices = [];
    for (var i = 0; i < model.vertices.length; i++) {
        vertices.push(MultiplyMV(transform, Vertex4(model.vertices[i])));
    }

    // Clip the entire model against each successive plane.
    var triangles = model.triangles.slice();
    for (var p = 0; p < clipping_planes.length; p++) {
        new_triangles = []
        for (var i = 0; i < triangles.length; i++) {
            ClipTriangle(triangles[i], clipping_planes[p], new_triangles, vertices);
        }
        triangles = new_triangles;
    }

    return Model(vertices, triangles, center, model.bounds_radius);
}


var RenderModel = function (model) {
    var projected = [];
    for (var i = 0; i < model.vertices.length; i++) {
        projected.push(ProjectVertex(Vertex4(model.vertices[i])));
    }
    for (var i = 0; i < model.triangles.length; i++) {
        RenderTriangle(model.triangles[i], model.vertices, projected);
    }
}


var RenderScene = function (camera, instances) {
    var cameraMatrix = MultiplyMM4(Transposed(camera.orientation), MakeTranslationMatrix(Multiply(-1, camera.position)));

    for (var i = 0; i < instances.length; i++) {
        var transform = MultiplyMM4(cameraMatrix, instances[i].transform);
        var clipped = TransformAndClip(camera.clipping_planes, instances[i].model, instances[i].scale, transform);
        if (clipped != null) {
            RenderModel(clipped);
        }
    }
}


var vertices = [
    Vertex(1, 1, 1),
    Vertex(-1, 1, 1),
    Vertex(-1, -1, 1),
    Vertex(1, -1, 1),
    Vertex(1, 1, -1),
    Vertex(-1, 1, -1),
    Vertex(-1, -1, -1),
    Vertex(1, -1, -1)
];

var RED = [255, 0, 0];
var GREEN = [0, 255, 0];
var BLUE = [0, 0, 255];
var YELLOW = [255, 255, 0];
var PURPLE = [255, 0, 255];
var CYAN = [0, 255, 255];

var triangles = [
    Triangle([0, 1, 2], RED),
    Triangle([0, 2, 3], RED),
    Triangle([1, 5, 6], YELLOW),
    Triangle([1, 6, 2], YELLOW),
    Triangle([2, 6, 7], CYAN),
    Triangle([2, 7, 3], CYAN),
    Triangle([4, 0, 3], GREEN),
    Triangle([4, 1, 0], PURPLE),
    Triangle([4, 3, 7], GREEN),
    Triangle([4, 5, 1], PURPLE),
    Triangle([5, 4, 7], BLUE),
    Triangle([5, 7, 6], BLUE),
];

Shuffle(triangles);

var cube = Model(vertices, triangles, Vertex(0, 0, 0), Math.sqrt(3));
var instances = [Instance(cube, Vertex(-1.5, 0, 7), Identity4x4, 0.75),
Instance(cube, Vertex(1.25, 2.5, 7.5), MakeOYRotationMatrix(195)),
];

var camera = Camera(Vertex(-3, 1, 2), MakeOYRotationMatrix(-30));

var s2 = Math.sqrt(2);
camera.clipping_planes = [
    Plane(Vertex(0, 0, 1), -1), // Near
    Plane(Vertex(s2, 0, s2), 0), // Left
    Plane(Vertex(-s2, 0, s2), 0), // Right
    Plane(Vertex(0, -s2, s2), 0), // Top
    Plane(Vertex(0, s2, s2), 0), // Bottom
];


var ShuffleCubeTriangles = function () {
    Shuffle(cube.triangles);
    Render();
}

var SetDepthEnabled = function (enabled) {
    depthBufferingEnabled = enabled;
    backfaceCullingEnabled = enabled;
    Render();
}

var SetOutlinesEnabled = function (enabled) {
    drawOutlines = enabled;
    Render();
}

var Render = function () {
    ClearAll();
    // This lets the browser clear the canvas before blocking to render the scene.
    setTimeout(function () {
        RenderScene(camera, instances);
        UpdateCanvas();
    }, 0);
}

Render();
