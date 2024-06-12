import {
	Mesh,
	BufferGeometry,
	TorusGeometry,
	Scene,
	Raycaster,
	MeshBasicMaterial,
	InterleavedBuffer,
	InterleavedBufferAttribute,
	InstancedMesh,
	Object3D
} from 'three';
import {
	acceleratedRaycast,
	computeBoundsTree,
	disposeBoundsTree,
	CENTER,
	SAH,
	AVERAGE,
} from '../src/index.js';
import { random, setSeed } from './utils.js';

Mesh.prototype.raycast = acceleratedRaycast;
BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;

describe( 'Random CENTER intersections', () => runRandomTests( { strategy: CENTER } ) );
describe( 'Random Interleaved CENTER intersections', () => runRandomTests( { strategy: CENTER, interleaved: true } ) );
describe( 'Random Indirect Buffer CENTER intersections', () => runRandomTests( { strategy: CENTER, indirect: true } ) );
describe( 'Random Instanced CENTER intersections', () => runRandomTests( { strategy: CENTER, instanced: true } ) );

describe( 'Random AVERAGE intersections', () => runRandomTests( { strategy: AVERAGE } ) );
describe( 'Random Interleaved AVERAGE intersections', () => runRandomTests( { strategy: AVERAGE, interleaved: true } ) );
describe( 'Random Indirect Buffer AVERAGE intersections', () => runRandomTests( { strategy: AVERAGE, indirect: true } ) );
describe( 'Random Instanced AVERAGE intersections', () => runRandomTests( { strategy: AVERAGE, instanced: true } ) );

describe( 'Random SAH intersections', () => runRandomTests( { strategy: SAH } ) );
describe( 'Random Interleaved SAH intersections', () => runRandomTests( { strategy: SAH, interleaved: true } ) );
describe( 'Random Indirect Buffer SAH intersections', () => runRandomTests( { strategy: SAH, indirect: true } ) );
describe( 'Random Instanced SAH intersections', () => runRandomTests( { strategy: SAH, instanced: true } ) );

describe( 'Random CENTER intersections with near', () => runRandomTests( { strategy: CENTER, near: 6 } ) );
describe( 'Random CENTER intersections with far', () => runRandomTests( { strategy: CENTER, far: 7 } ) );
describe( 'Random CENTER intersections with near and far', () => runRandomTests( { strategy: CENTER, near: 6, far: 7 } ) );

function runRandomTests( options ) {

	const transformSeed = Math.floor( Math.random() * 1e10 );
	describe( `Transform Seed : ${ transformSeed }`, () => {

		let scene,
			raycaster,
			ungroupedGeometry,
			ungroupedBvh,
			groupedGeometry,
			groupedBvh;

		beforeAll( () => {

			ungroupedGeometry = new TorusGeometry( 1, 1, 40, 10 );
			groupedGeometry = new TorusGeometry( 1, 1, 40, 10 );

			if ( options.interleaved ) {

				ungroupedGeometry.setAttribute( 'position', createInterleavedPositionBuffer( ungroupedGeometry.attributes.position ) );
				groupedGeometry.setAttribute( 'position', createInterleavedPositionBuffer( groupedGeometry.attributes.position ) );

			}

			const groupCount = 10;
			const groupSize = groupedGeometry.index.array.length / groupCount;

			for ( let g = 0; g < groupCount; g ++ ) {

				const groupStart = g * groupSize;
				groupedGeometry.addGroup( groupStart, groupSize, 0 );

			}

			groupedGeometry.computeBoundsTree( options );
			ungroupedGeometry.computeBoundsTree( options );

			ungroupedBvh = ungroupedGeometry.boundsTree;
			groupedBvh = groupedGeometry.boundsTree;

			scene = new Scene();
			raycaster = new Raycaster();

			if ( options.near !== undefined ) {

				raycaster.near = options.near;

			}

			if ( options.far !== undefined ) {

				raycaster.far = options.far;

			}

			setSeed( transformSeed );
			random(); // call random() to seed with a larger value

			if ( options.instanced ) {

				const geo = groupedGeometry; // ungroupedGeometry not used...
				const instancedMesh = new InstancedMesh( geo, new MeshBasicMaterial(), 10 );

				updateMatrix( instancedMesh );

				instancedMesh.updateMatrixWorld( true );

				scene.add( instancedMesh );

				const tempObj = new Object3D();

				for ( var i = 0; i < 10; i ++ ) {

					updateMatrix( tempObj );
					instancedMesh.setMatrixAt( i, tempObj.matrix );

				}

			} else {

				for ( var i = 0; i < 10; i ++ ) {

					let geo = i % 2 ? groupedGeometry : ungroupedGeometry;
					let mesh = new Mesh( geo, new MeshBasicMaterial() );

					updateMatrix( mesh );
					mesh.updateMatrixWorld( true );
					scene.add( mesh );

				}

			}

		} );

		for ( let i = 0; i < 100; i ++ ) {

			const raySeed = Math.floor( Math.random() * 1e10 );
			it( `Cast ${ i } Seed : ${ raySeed }`, () => {

				setSeed( raySeed );
				random(); // call random() to seed with a larger value

				raycaster.firstHitOnly = false;
				raycaster.ray.origin.set( random() * 10, random() * 10, random() * 10 );
				raycaster.ray.direction.copy( raycaster.ray.origin ).multiplyScalar( - 1 ).normalize();

				ungroupedGeometry.boundsTree = ungroupedBvh;
				groupedGeometry.boundsTree = groupedBvh;
				const bvhHits = raycaster.intersectObject( scene, true );

				raycaster.firstHitOnly = true;
				const firstHit = raycaster.intersectObject( scene, true );

				ungroupedGeometry.boundsTree = null;
				groupedGeometry.boundsTree = null;
				const ogHits = raycaster.intersectObject( scene, true );

				expect( ogHits ).toEqual( bvhHits );
				expect( firstHit[ 0 ] ).toEqual( ogHits[ 0 ] );

			} );

		}

	} );

}

function createInterleavedPositionBuffer( bufferAttribute ) {

	const array = bufferAttribute.array;
	const newArray = new array.constructor( array.length * 2 );
	const newBuffer = new InterleavedBufferAttribute( new InterleavedBuffer( newArray, 6 ), 3, 3, bufferAttribute.normalized );
	for ( let i = 0; i < bufferAttribute.count; i ++ ) {

		newBuffer.setXYZ(
			i,
			bufferAttribute.getX( i ),
			bufferAttribute.getY( i ),
			bufferAttribute.getZ( i ),
		);

	}

	return newBuffer;

}


function updateMatrix( target ) {

	target.rotation.x = random() * 10;
	target.rotation.y = random() * 10;
	target.rotation.z = random() * 10;

	target.position.x = random();
	target.position.y = random();
	target.position.z = random();

	target.scale.x = random() * 2 - 1;
	target.scale.y = random() * 2 - 1;
	target.scale.z = random() * 2 - 1;

	target.updateMatrix( true );

}
