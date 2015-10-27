package com.badlogic.gdx.tests.bullet;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btAxisSweep3;
import com.badlogic.gdx.physics.bullet.collision.btBroadphasePair;
import com.badlogic.gdx.physics.bullet.collision.btBroadphasePairArray;
import com.badlogic.gdx.physics.bullet.collision.btCapsuleShape;
import com.badlogic.gdx.physics.bullet.collision.btCollisionDispatcher;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;
import com.badlogic.gdx.physics.bullet.collision.btCollisionWorld;
import com.badlogic.gdx.physics.bullet.collision.btConvexShape;
import com.badlogic.gdx.physics.bullet.collision.btDefaultCollisionConfiguration;
import com.badlogic.gdx.physics.bullet.collision.btGhostPairCallback;
import com.badlogic.gdx.physics.bullet.collision.btManifoldPoint;
import com.badlogic.gdx.physics.bullet.collision.btPairCachingGhostObject;
import com.badlogic.gdx.physics.bullet.collision.btPersistentManifold;
import com.badlogic.gdx.physics.bullet.collision.btPersistentManifoldArray;

/** @author eadle */
public class GhostObjectTest extends BaseBulletTest {

    ModelInstance capsule;
    btConvexShape convexShape;
    btPairCachingGhostObject ghostObject;

    // avoid garbage collection
    btPersistentManifoldArray manifoldArray;
    Vector3 minAabb = new Vector3();
    Vector3 maxAabb = new Vector3();
    Vector3 recovery = new Vector3();
    Vector3 correction = new Vector3();
    Vector3 surfaceNormal = new Vector3();

    public BulletWorld createWorld() {
        btDefaultCollisionConfiguration collisionConfig = new btDefaultCollisionConfiguration();
        btCollisionDispatcher dispatcher = new btCollisionDispatcher(collisionConfig);
        btAxisSweep3 sweep = new btAxisSweep3(new Vector3(-1000, -1000, -1000), new Vector3(1000, 1000, 1000));
        sweep.getOverlappingPairCache().setInternalGhostPairCallback(new btGhostPairCallback());
        btCollisionWorld collisionWorld = new btCollisionWorld(dispatcher, sweep, collisionConfig);
        return new BulletWorld(collisionConfig, dispatcher, sweep, null, collisionWorld);
    }

    @Override
    public void create() {
        super.create();

        instructions = "Long press to toggle debug mode\nSwipe for next test\nCtrl+drag to rotate\nScroll to zoom";

        camera.position.set(2.f, 2.f, 2.f);
        camera.update();

        setupGhostObject();

        Model groundModel = world.getConstructor("ground").model;
        Model boxModel = world.getConstructor("box").model;

        world.addConstructor("collisionGround", new BulletConstructor(groundModel));
        world.addConstructor("collisionBox", new BulletConstructor(boxModel));

        world.add("collisionGround", 0f, 0f, 0f).setColor(0.25f + 0.5f * (float) Math.random(),
                0.25f + 0.5f * (float) Math.random(), 0.25f + 0.5f * (float) Math.random(), 1f);

    }

    private void setupGhostObject() {

        manifoldArray = new btPersistentManifoldArray();

        // capsule dimensions
        float radius = 0.25f;
        float height = 1.f;

        Model capsuleModel = modelBuilder.createCapsule(radius, height, 10,
                new Material(ColorAttribute.createDiffuse(Color.GREEN)),
                VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);
        disposables.add(capsuleModel);

        capsule = new ModelInstance(capsuleModel);
        convexShape = new btCapsuleShape(radius, height - 2.f*radius);
        ghostObject = new btPairCachingGhostObject();
        ghostObject.setCollisionShape(convexShape);
        ghostObject.setCollisionFlags(btCollisionObject.CollisionFlags.CF_CHARACTER_OBJECT);

        world.collisionWorld.addCollisionObject(ghostObject);
    }

    @Override
    public void update() {
        super.update();

        // Not using dynamics, so update the collision world manually
        if (world.performanceCounter != null) world.performanceCounter.start();
        world.collisionWorld.performDiscreteCollisionDetection();
        if (world.performanceCounter != null) world.performanceCounter.stop();

        // check for collision -- keeps track of its own overlapping pairs
        if (ghostObject.getOverlappingPairCache().getNumOverlappingPairs() > 0) {
            recoverFromPenetration();
        }

    }

    private void recoverFromPenetration() {

        convexShape.getAabb(ghostObject.getWorldTransform(), minAabb, maxAabb);
        world.collisionWorld.getBroadphase().setAabb(ghostObject.getBroadphaseHandle(), minAabb,
                maxAabb, world.collisionWorld.getDispatcher());
        world.collisionWorld.getDispatcher().dispatchAllCollisionPairs(ghostObject.getOverlappingPairCache(),
                world.collisionWorld.getDispatchInfo(), world.collisionWorld.getDispatcher());

        // clear the correction vector
        correction.setZero();

        // iterate through all collision pairs
        btBroadphasePairArray pairArray = ghostObject.getOverlappingPairCache().getOverlappingPairArray();
        for (int i = 0; i < pairArray.size(); i++) {
            btBroadphasePair collisionPair = pairArray.at(i);

            // Java supports short-circuit evaluations
            if (0 == collisionPair.getCPointer() || null == collisionPair.getAlgorithm()) {
                continue;
            }

            manifoldArray.resize(0);
            collisionPair.getAlgorithm().getAllContactManifolds(manifoldArray);

            for (int j = 0; j < manifoldArray.size(); j++) {
                btPersistentManifold manifold = manifoldArray.at(j);
                boolean isFirstBody = (manifold.getBody0().getCPointer() == ghostObject.getCPointer());
                float directionSign = isFirstBody ? -1.f : 1.f;

                for (int p = 0; p < manifold.getNumContacts(); p++) {
                    btManifoldPoint pt = manifold.getContactPoint(p);
                    float distance = pt.getDistance();

                    // negative distance means there was a penetration
                    if (distance < 0.f) {
                        // get the recovery vector
                        pt.getNormalWorldOnB(recovery);
                        recovery.scl(directionSign*distance);
                        // lazy recovery
                        if (Math.abs(recovery.x) > Math.abs(correction.x))
                            correction.x = recovery.x;
                        if (Math.abs(recovery.y) > Math.abs(correction.y))
                            correction.y = recovery.y;
                        if (Math.abs(recovery.z) > Math.abs(correction.z))
                            correction.z = recovery.z;
                    }

                }
            }
        }

        ghostObject.getWorldTransform(capsule.transform);
        capsule.transform.translate(correction);
        ghostObject.setWorldTransform(capsule.transform);

    }

    @Override
    protected void renderWorld() {
        modelBatch.begin(camera);
        modelBatch.render(capsule, environment);
        modelBatch.end();
        super.renderWorld();
    }

    @Override
    public void dispose() {
        super.dispose();

        convexShape.dispose();
        ghostObject.dispose();
    }
}
