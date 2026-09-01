import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';

const CONFIG = {
    offsetX: 0.82,
    offsetY: -0.40,
    fitPadding: 3.3,
    basePose: [0.4, -0.5, 0, -2.0, 0, 1.6, 0.78],
    ikChain: ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7'],
    ikIterations: 4,
    ikDamping: 0.4,
    targetEase: 0.12,
    eefSize: 0.14,
    eefFaceOffset: [0, Math.PI, 0],
};

const canvas = document.getElementById('robot-canvas');
const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
const isMobile = window.matchMedia('(max-width: 768px)').matches;

if (canvas && !reduceMotion && !isMobile) {
    initRobot(canvas).catch(err => console.error('[robot] init failed:', err));
}

function createTactileTexture() {
    var size = 256;
    var cvs = document.createElement('canvas');
    cvs.width = size; cvs.height = size;
    var ctx = cvs.getContext('2d');
    var texture = new THREE.CanvasTexture(cvs);

    var blobs = [
        { bx: 128, by: 80,  r: 60, c: [255, 90, 60],  p: 0 },
        { bx: 85,  by: 145, r: 48, c: [255, 200, 50],  p: 1.5 },
        { bx: 170, by: 110, r: 44, c: [80, 230, 130],  p: 3.0 },
        { bx: 128, by: 170, r: 52, c: [70, 140, 255],  p: 4.5 },
        { bx: 138, by: 120, r: 30, c: [255, 255, 210],  p: 2.0 },
    ];

    function paint() {
        var t = performance.now() * 0.001;
        ctx.clearRect(0, 0, size, size);

        ctx.fillStyle = '#5a78a8';
        ctx.beginPath();
        ctx.arc(size / 2, size / 2, size / 2, 0, Math.PI * 2);
        ctx.fill();

        blobs.forEach(function (b) {
            var tt = t + b.p;
            var x = b.bx + Math.sin(tt * 0.7) * 10;
            var y = b.by + Math.cos(tt * 0.5) * 10;
            var r = b.r + Math.sin(tt * 1.1) * 6;
            var a = 0.8 + Math.sin(tt * 0.9) * 0.15;
            var g = ctx.createRadialGradient(x, y, 0, x, y, r);
            g.addColorStop(0, 'rgba(' + b.c[0] + ',' + b.c[1] + ',' + b.c[2] + ',' + a + ')');
            g.addColorStop(0.55, 'rgba(' + b.c[0] + ',' + b.c[1] + ',' + b.c[2] + ',' + (a * 0.4) + ')');
            g.addColorStop(1, 'transparent');
            ctx.fillStyle = g;
            ctx.beginPath();
            ctx.arc(x, y, r, 0, Math.PI * 2);
            ctx.fill();
        });

        ctx.globalCompositeOperation = 'destination-in';
        ctx.fillStyle = '#fff';
        ctx.beginPath();
        ctx.arc(size / 2, size / 2, size / 2 - 2, 0, Math.PI * 2);
        ctx.fill();
        ctx.globalCompositeOperation = 'source-over';

        texture.needsUpdate = true;
    }

    paint();
    setInterval(paint, 120);
    return texture;
}

function createTactileSensor() {
    var group = new THREE.Group();

    var bodyGeo = new THREE.CylinderGeometry(0.027, 0.030, 0.045, 32);
    bodyGeo.rotateX(Math.PI / 2);
    var bodyMat = new THREE.MeshBasicMaterial({ color: 0x555560 });
    group.add(new THREE.Mesh(bodyGeo, bodyMat));

    var gelGeo = new THREE.CircleGeometry(0.026, 48);
    var gelMat = new THREE.MeshBasicMaterial({
        map: createTactileTexture(),
        side: THREE.DoubleSide,
        transparent: true
    });
    var gel = new THREE.Mesh(gelGeo, gelMat);
    gel.position.z = 0.026;
    group.add(gel);

    var rimGeo = new THREE.TorusGeometry(0.026, 0.003, 8, 48);
    var rimMat = new THREE.MeshBasicMaterial({ color: 0x999999 });
    var rim = new THREE.Mesh(rimGeo, rimMat);
    rim.position.z = 0.026;
    group.add(rim);

    var ledColors = [0xff4444, 0x44ff66, 0x4488ff];
    ledColors.forEach(function (c, i) {
        var led = new THREE.Mesh(
            new THREE.SphereGeometry(0.004, 8, 8),
            new THREE.MeshBasicMaterial({ color: c })
        );
        var ang = (i / 3) * Math.PI * 2 - Math.PI / 2;
        led.position.set(Math.cos(ang) * 0.022, Math.sin(ang) * 0.022, 0.020);
        group.add(led);
    });

    var box = new THREE.Box3().setFromObject(group);
    var maxDim = Math.max.apply(null, box.getSize(new THREE.Vector3()).toArray());
    group.scale.setScalar(CONFIG.eefSize / (maxDim || 1));

    return group;
}

async function initRobot(canvas) {
    var scene = new THREE.Scene();
    var renderer = new THREE.WebGLRenderer({ canvas: canvas, alpha: true, antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1));
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.outputColorSpace = THREE.SRGBColorSpace;

    var camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.01, 100);
    scene.add(new THREE.HemisphereLight(0xffffff, 0xdfe6ef, 1.6));
    scene.add(new THREE.AmbientLight(0xffffff, 0.55));

    var pivot = new THREE.Group();
    scene.add(pivot);

    var robot = null;
    var eef = null;
    var eefModel = null;
    var framed = false;
    var pointer = { x: 0, y: 0 };

    var manager = new THREE.LoadingManager();
    var loader = new URDFLoader(manager);
    loader.loadMeshCb = function (path, mgr, done) {
        new ColladaLoader(mgr).load(
            path,
            function (collada) { done(collada.scene); },
            undefined,
            function (err) { console.error('[robot] mesh failed:', path, err); done(null, err); }
        );
    };
    var allLoaded = new Promise(function (res) { manager.onLoad = res; });

    robot = await new Promise(function (resolve, reject) {
        loader.load('./static/models/franka/panda_arm.urdf', resolve, undefined, reject);
    });

    robot.rotation.x = -Math.PI / 2;
    CONFIG.basePose.forEach(function (v, i) {
        var n = 'panda_joint' + (i + 1);
        if (robot.joints[n]) robot.setJointValue(n, v);
    });
    pivot.add(robot);

    robot.traverse(function (o) {
        if (o.material) {
            (Array.isArray(o.material) ? o.material : [o.material]).forEach(function (m) {
                if ('shininess' in m) m.shininess = 0;
                if (m.specular) m.specular.setHex(0x000000);
            });
        }
    });

    var eefLink = robot.links['panda_link8'] || robot.links['panda_link7'];
    try { eefModel = createTactileSensor(); scene.add(eefModel); }
    catch (err) { console.error('[robot] sensor model failed:', err); }
    eef = new THREE.Object3D();
    eef.position.set(0, 0, 0.06);
    eefLink.add(eef);

    await allLoaded;
    {
        robot.updateMatrixWorld(true);
        var box = new THREE.Box3().setFromObject(pivot);
        var size = box.getSize(new THREE.Vector3());
        var center = box.getCenter(new THREE.Vector3());
        robot.position.sub(center);

        var maxDim = Math.max(size.x, size.y, size.z) || 1;
        var fov = camera.fov * Math.PI / 180;
        var dist = (maxDim / 2) / Math.tan(fov / 2) * CONFIG.fitPadding;

        camera.position.set(0, 0, dist);
        camera.lookAt(0, 0, 0);

        var vH = 2 * dist * Math.tan(fov / 2);
        var vW = vH * camera.aspect;
        pivot.position.x = (vW / 2) * CONFIG.offsetX;
        pivot.position.y = (vH / 2) * CONFIG.offsetY;

        framed = true;
    }

    window.addEventListener('pointermove', function (e) {
        pointer.x = (e.clientX / window.innerWidth) * 2 - 1;
        pointer.y = (e.clientY / window.innerHeight) * 2 - 1;
    });

    window.addEventListener('resize', function () {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });

    var target = new THREE.Vector3();
    var smooth = new THREE.Vector3(0, 0, 0);
    var _jp = new THREE.Vector3(), _ee = new THREE.Vector3(), _ax = new THREE.Vector3();
    var _a = new THREE.Vector3(), _b = new THREE.Vector3(), _c = new THREE.Vector3();
    var seeded = false;

    function mouseTarget(out) {
        out.set(pointer.x, -pointer.y, 0.5).unproject(camera);
        out.sub(camera.position).normalize();
        var t = (0 - camera.position.z) / out.z;
        out.multiplyScalar(t).add(camera.position);
    }

    function solveIK() {
        for (var it = 0; it < CONFIG.ikIterations; it++) {
            for (var k = CONFIG.ikChain.length - 1; k >= 0; k--) {
                var joint = robot.joints[CONFIG.ikChain[k]];
                if (!joint) continue;
                robot.updateMatrixWorld(true);
                joint.getWorldPosition(_jp);
                eef.getWorldPosition(_ee);
                _ax.copy(joint.axis).transformDirection(joint.matrixWorld).normalize();
                _a.subVectors(_ee, _jp);
                _a.addScaledVector(_ax, -_a.dot(_ax));
                _b.subVectors(smooth, _jp);
                _b.addScaledVector(_ax, -_b.dot(_ax));
                if (_a.lengthSq() < 1e-8 || _b.lengthSq() < 1e-8) continue;
                _a.normalize(); _b.normalize();
                var ang = Math.acos(THREE.MathUtils.clamp(_a.dot(_b), -1, 1));
                if (_c.crossVectors(_a, _b).dot(_ax) < 0) ang = -ang;
                joint.setJointValue(joint.angle + ang * CONFIG.ikDamping);
            }
        }
    }

    var running = true;
    document.addEventListener('visibilitychange', function () {
        running = !document.hidden;
        if (running) animate();
    });

    function animate() {
        if (!running) return;
        requestAnimationFrame(animate);
        if (robot && framed) {
            mouseTarget(target);
            if (!seeded) { smooth.copy(target); seeded = true; }
            smooth.lerp(target, CONFIG.targetEase);
            solveIK();
            if (eefModel) {
                eef.getWorldPosition(_ee);
                eefModel.position.copy(_ee);
                eefModel.lookAt(camera.position);
                var fo = CONFIG.eefFaceOffset;
                eefModel.rotateX(fo[0]);
                eefModel.rotateY(fo[1]);
                eefModel.rotateZ(fo[2]);
            }
        }
        renderer.render(scene, camera);
    }
    animate();
}
