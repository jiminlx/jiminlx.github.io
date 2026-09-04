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
    fingerOpen: 0.02,
};

const canvas = document.getElementById('robot-canvas');
const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
const isMobile = window.matchMedia('(max-width: 768px)').matches;

if (canvas && !reduceMotion && !isMobile) {
    initRobot(canvas).catch(err => console.error('[robot] init failed:', err));
}

function tweakMaterials(obj) {
    obj.traverse(function (o) {
        if (!o.material) return;
        (Array.isArray(o.material) ? o.material : [o.material]).forEach(function (m) {
            if ('shininess' in m) m.shininess = 0;
            if (m.specular) m.specular.setHex(0x000000);
        });
    });
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
    var framed = false;
    var pointer = { x: 0, y: 0 };

    var manager = new THREE.LoadingManager();
    var colladaLoader = new ColladaLoader(manager);
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

    var eefLink = robot.links['panda_link8'] || robot.links['panda_link7'];

    var handGroup = new THREE.Group();
    handGroup.rotation.z = -Math.PI / 4;
    eefLink.add(handGroup);

    colladaLoader.load('./static/models/franka/meshes/visual/hand.dae', function (c) {
        tweakMaterials(c.scene);
        handGroup.add(c.scene);
    });

    var leftFinger = new THREE.Group();
    leftFinger.position.set(0, CONFIG.fingerOpen, 0.0584);
    handGroup.add(leftFinger);

    var rightFinger = new THREE.Group();
    rightFinger.position.set(0, -CONFIG.fingerOpen, 0.0584);
    handGroup.add(rightFinger);

    colladaLoader.load('./static/models/franka/meshes/visual/finger.dae', function (c) {
        tweakMaterials(c.scene);
        leftFinger.add(c.scene);
    });

    colladaLoader.load('./static/models/franka/meshes/visual/finger.dae', function (c) {
        tweakMaterials(c.scene);
        c.scene.rotation.z = Math.PI;
        rightFinger.add(c.scene);
    });

    eef = new THREE.Object3D();
    eef.position.set(0, 0, 0.12);
    eefLink.add(eef);

    await allLoaded;

    tweakMaterials(robot);

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
        }
        renderer.render(scene, camera);
    }
    animate();
}
