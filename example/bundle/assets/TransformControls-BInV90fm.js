import{k as Mt,V as a,O as et,aq as g,n as ut,f as mt,w as vt,l as w,X as m,i as ot,ag as st,M as n,aI as U,ak as I,e as Et,aj as q,s as It,t as Qt,a7 as Xt}from"./ExtendedTriangle-CtXgmvQ9.js";const A=new Mt,d=new a,Q=new a,h=new g,at={X:new a(1,0,0),Y:new a(0,1,0),Z:new a(0,0,1)},$={type:"change"},rt={type:"mouseDown",mode:null},lt={type:"mouseUp",mode:null},ht={type:"objectChange"};class qt extends et{constructor(i,s){super(),s===void 0&&(console.warn('THREE.TransformControls: The second parameter "domElement" is now mandatory.'),s=document),this.isTransformControls=!0,this.visible=!1,this.domElement=s,this.domElement.style.touchAction="none";const o=new zt;this._gizmo=o,this.add(o);const e=new Dt;this._plane=e,this.add(e);const l=this;function t(u,b){let H=b;Object.defineProperty(l,u,{get:function(){return H!==void 0?H:b},set:function(Y){H!==Y&&(H=Y,e[u]=Y,o[u]=Y,l.dispatchEvent({type:u+"-changed",value:Y}),l.dispatchEvent($))}}),l[u]=b,e[u]=b,o[u]=b}t("camera",i),t("object",void 0),t("enabled",!0),t("axis",null),t("mode","translate"),t("translationSnap",null),t("rotationSnap",null),t("scaleSnap",null),t("space","world"),t("size",1),t("dragging",!1),t("showX",!0),t("showY",!0),t("showZ",!0);const c=new a,y=new a,X=new g,T=new g,D=new a,j=new g,it=new a,C=new a,M=new a,v=0,S=new a;t("worldPosition",c),t("worldPositionStart",y),t("worldQuaternion",X),t("worldQuaternionStart",T),t("cameraPosition",D),t("cameraQuaternion",j),t("pointStart",it),t("pointEnd",C),t("rotationAxis",M),t("rotationAngle",v),t("eye",S),this._offset=new a,this._startNorm=new a,this._endNorm=new a,this._cameraScale=new a,this._parentPosition=new a,this._parentQuaternion=new g,this._parentQuaternionInv=new g,this._parentScale=new a,this._worldScaleStart=new a,this._worldQuaternionInv=new g,this._worldScale=new a,this._positionStart=new a,this._quaternionStart=new g,this._scaleStart=new a,this._getPointer=Yt.bind(this),this._onPointerDown=Zt.bind(this),this._onPointerHover=At.bind(this),this._onPointerMove=Tt.bind(this),this._onPointerUp=Ht.bind(this),this.domElement.addEventListener("pointerdown",this._onPointerDown),this.domElement.addEventListener("pointermove",this._onPointerHover),this.domElement.addEventListener("pointerup",this._onPointerUp)}updateMatrixWorld(i){this.object!==void 0&&(this.object.updateMatrixWorld(),this.object.parent===null?console.error("TransformControls: The attached 3D object must be a part of the scene graph."):this.object.parent.matrixWorld.decompose(this._parentPosition,this._parentQuaternion,this._parentScale),this.object.matrixWorld.decompose(this.worldPosition,this.worldQuaternion,this._worldScale),this._parentQuaternionInv.copy(this._parentQuaternion).invert(),this._worldQuaternionInv.copy(this.worldQuaternion).invert()),this.camera.updateMatrixWorld(),this.camera.matrixWorld.decompose(this.cameraPosition,this.cameraQuaternion,this._cameraScale),this.camera.isOrthographicCamera?this.camera.getWorldDirection(this.eye).negate():this.eye.copy(this.cameraPosition).sub(this.worldPosition).normalize(),super.updateMatrixWorld(i)}pointerHover(i){if(this.object===void 0||this.dragging===!0)return;i!==null&&A.setFromCamera(i,this.camera);const s=tt(this._gizmo.picker[this.mode],A);s?this.axis=s.object.name:this.axis=null}pointerDown(i){if(!(this.object===void 0||this.dragging===!0||i!=null&&i.button!==0)&&this.axis!==null){i!==null&&A.setFromCamera(i,this.camera);const s=tt(this._plane,A,!0);s&&(this.object.updateMatrixWorld(),this.object.parent.updateMatrixWorld(),this._positionStart.copy(this.object.position),this._quaternionStart.copy(this.object.quaternion),this._scaleStart.copy(this.object.scale),this.object.matrixWorld.decompose(this.worldPositionStart,this.worldQuaternionStart,this._worldScaleStart),this.pointStart.copy(s.point).sub(this.worldPositionStart)),this.dragging=!0,rt.mode=this.mode,this.dispatchEvent(rt)}}pointerMove(i){const s=this.axis,o=this.mode,e=this.object;let l=this.space;if(o==="scale"?l="local":(s==="E"||s==="XYZE"||s==="XYZ")&&(l="world"),e===void 0||s===null||this.dragging===!1||i!==null&&i.button!==-1)return;i!==null&&A.setFromCamera(i,this.camera);const t=tt(this._plane,A,!0);if(t){if(this.pointEnd.copy(t.point).sub(this.worldPositionStart),o==="translate")this._offset.copy(this.pointEnd).sub(this.pointStart),l==="local"&&s!=="XYZ"&&this._offset.applyQuaternion(this._worldQuaternionInv),s.indexOf("X")===-1&&(this._offset.x=0),s.indexOf("Y")===-1&&(this._offset.y=0),s.indexOf("Z")===-1&&(this._offset.z=0),l==="local"&&s!=="XYZ"?this._offset.applyQuaternion(this._quaternionStart).divide(this._parentScale):this._offset.applyQuaternion(this._parentQuaternionInv).divide(this._parentScale),e.position.copy(this._offset).add(this._positionStart),this.translationSnap&&(l==="local"&&(e.position.applyQuaternion(h.copy(this._quaternionStart).invert()),s.search("X")!==-1&&(e.position.x=Math.round(e.position.x/this.translationSnap)*this.translationSnap),s.search("Y")!==-1&&(e.position.y=Math.round(e.position.y/this.translationSnap)*this.translationSnap),s.search("Z")!==-1&&(e.position.z=Math.round(e.position.z/this.translationSnap)*this.translationSnap),e.position.applyQuaternion(this._quaternionStart)),l==="world"&&(e.parent&&e.position.add(d.setFromMatrixPosition(e.parent.matrixWorld)),s.search("X")!==-1&&(e.position.x=Math.round(e.position.x/this.translationSnap)*this.translationSnap),s.search("Y")!==-1&&(e.position.y=Math.round(e.position.y/this.translationSnap)*this.translationSnap),s.search("Z")!==-1&&(e.position.z=Math.round(e.position.z/this.translationSnap)*this.translationSnap),e.parent&&e.position.sub(d.setFromMatrixPosition(e.parent.matrixWorld))));else if(o==="scale"){if(s.search("XYZ")!==-1){let c=this.pointEnd.length()/this.pointStart.length();this.pointEnd.dot(this.pointStart)<0&&(c*=-1),Q.set(c,c,c)}else d.copy(this.pointStart),Q.copy(this.pointEnd),d.applyQuaternion(this._worldQuaternionInv),Q.applyQuaternion(this._worldQuaternionInv),Q.divide(d),s.search("X")===-1&&(Q.x=1),s.search("Y")===-1&&(Q.y=1),s.search("Z")===-1&&(Q.z=1);e.scale.copy(this._scaleStart).multiply(Q),this.scaleSnap&&(s.search("X")!==-1&&(e.scale.x=Math.round(e.scale.x/this.scaleSnap)*this.scaleSnap||this.scaleSnap),s.search("Y")!==-1&&(e.scale.y=Math.round(e.scale.y/this.scaleSnap)*this.scaleSnap||this.scaleSnap),s.search("Z")!==-1&&(e.scale.z=Math.round(e.scale.z/this.scaleSnap)*this.scaleSnap||this.scaleSnap))}else if(o==="rotate"){this._offset.copy(this.pointEnd).sub(this.pointStart);const c=20/this.worldPosition.distanceTo(d.setFromMatrixPosition(this.camera.matrixWorld));let y=!1;s==="XYZE"?(this.rotationAxis.copy(this._offset).cross(this.eye).normalize(),this.rotationAngle=this._offset.dot(d.copy(this.rotationAxis).cross(this.eye))*c):(s==="X"||s==="Y"||s==="Z")&&(this.rotationAxis.copy(at[s]),d.copy(at[s]),l==="local"&&d.applyQuaternion(this.worldQuaternion),d.cross(this.eye),d.length()===0?y=!0:this.rotationAngle=this._offset.dot(d.normalize())*c),(s==="E"||y)&&(this.rotationAxis.copy(this.eye),this.rotationAngle=this.pointEnd.angleTo(this.pointStart),this._startNorm.copy(this.pointStart).normalize(),this._endNorm.copy(this.pointEnd).normalize(),this.rotationAngle*=this._endNorm.cross(this._startNorm).dot(this.eye)<0?1:-1),this.rotationSnap&&(this.rotationAngle=Math.round(this.rotationAngle/this.rotationSnap)*this.rotationSnap),l==="local"&&s!=="E"&&s!=="XYZE"?(e.quaternion.copy(this._quaternionStart),e.quaternion.multiply(h.setFromAxisAngle(this.rotationAxis,this.rotationAngle)).normalize()):(this.rotationAxis.applyQuaternion(this._parentQuaternionInv),e.quaternion.copy(h.setFromAxisAngle(this.rotationAxis,this.rotationAngle)),e.quaternion.multiply(this._quaternionStart).normalize())}this.dispatchEvent($),this.dispatchEvent(ht)}}pointerUp(i){i!==null&&i.button!==0||(this.dragging&&this.axis!==null&&(lt.mode=this.mode,this.dispatchEvent(lt)),this.dragging=!1,this.axis=null)}dispose(){this.domElement.removeEventListener("pointerdown",this._onPointerDown),this.domElement.removeEventListener("pointermove",this._onPointerHover),this.domElement.removeEventListener("pointermove",this._onPointerMove),this.domElement.removeEventListener("pointerup",this._onPointerUp),this.traverse(function(i){i.geometry&&i.geometry.dispose(),i.material&&i.material.dispose()})}attach(i){return this.object=i,this.visible=!0,this}detach(){return this.object=void 0,this.visible=!1,this.axis=null,this}reset(){this.enabled&&this.dragging&&(this.object.position.copy(this._positionStart),this.object.quaternion.copy(this._quaternionStart),this.object.scale.copy(this._scaleStart),this.dispatchEvent($),this.dispatchEvent(ht),this.pointStart.copy(this.pointEnd))}getRaycaster(){return A}getMode(){return this.mode}setMode(i){this.mode=i}setTranslationSnap(i){this.translationSnap=i}setRotationSnap(i){this.rotationSnap=i}setScaleSnap(i){this.scaleSnap=i}setSize(i){this.size=i}setSpace(i){this.space=i}}function Yt(p){if(this.domElement.ownerDocument.pointerLockElement)return{x:0,y:0,button:p.button};{const i=this.domElement.getBoundingClientRect();return{x:(p.clientX-i.left)/i.width*2-1,y:-(p.clientY-i.top)/i.height*2+1,button:p.button}}}function At(p){if(this.enabled)switch(p.pointerType){case"mouse":case"pen":this.pointerHover(this._getPointer(p));break}}function Zt(p){this.enabled&&(document.pointerLockElement||this.domElement.setPointerCapture(p.pointerId),this.domElement.addEventListener("pointermove",this._onPointerMove),this.pointerHover(this._getPointer(p)),this.pointerDown(this._getPointer(p)))}function Tt(p){this.enabled&&this.pointerMove(this._getPointer(p))}function Ht(p){this.enabled&&(this.domElement.releasePointerCapture(p.pointerId),this.domElement.removeEventListener("pointermove",this._onPointerMove),this.pointerUp(this._getPointer(p)))}function tt(p,i,s){const o=i.intersectObject(p,!0);for(let e=0;e<o.length;e++)if(o[e].object.visible||s)return o[e];return!1}const N=new Xt,r=new a(0,1,0),ct=new a(0,0,0),pt=new ut,V=new g,K=new g,x=new a,dt=new ut,O=new a(1,0,0),Z=new a(0,1,0),R=new a(0,0,1),J=new a,k=new a,L=new a;class zt extends et{constructor(){super(),this.isTransformControlsGizmo=!0,this.type="TransformControlsGizmo";const i=new mt({depthTest:!1,depthWrite:!1,fog:!1,toneMapped:!1,transparent:!0}),s=new vt({depthTest:!1,depthWrite:!1,fog:!1,toneMapped:!1,transparent:!0}),o=i.clone();o.opacity=.15;const e=s.clone();e.opacity=.5;const l=i.clone();l.color.setHex(16711680);const t=i.clone();t.color.setHex(65280);const c=i.clone();c.color.setHex(255);const y=i.clone();y.color.setHex(16711680),y.opacity=.5;const X=i.clone();X.color.setHex(65280),X.opacity=.5;const T=i.clone();T.color.setHex(255),T.opacity=.5;const D=i.clone();D.opacity=.25;const j=i.clone();j.color.setHex(16776960),j.opacity=.25,i.clone().color.setHex(16776960);const C=i.clone();C.color.setHex(7895160);const M=new w(0,.04,.1,12);M.translate(0,.05,0);const v=new m(.08,.08,.08);v.translate(0,.04,0);const S=new ot;S.setAttribute("position",new st([0,0,0,1,0,0],3));const u=new w(.0075,.0075,.5,3);u.translate(0,.25,0);function b(_,W){const P=new q(_,.0075,3,64,W*Math.PI*2);return P.rotateY(Math.PI/2),P.rotateX(Math.PI/2),P}function H(){const _=new ot;return _.setAttribute("position",new st([0,0,0,1,1,1],3)),_}const Y={X:[[new n(M,l),[.5,0,0],[0,0,-Math.PI/2]],[new n(M,l),[-.5,0,0],[0,0,Math.PI/2]],[new n(u,l),[0,0,0],[0,0,-Math.PI/2]]],Y:[[new n(M,t),[0,.5,0]],[new n(M,t),[0,-.5,0],[Math.PI,0,0]],[new n(u,t)]],Z:[[new n(M,c),[0,0,.5],[Math.PI/2,0,0]],[new n(M,c),[0,0,-.5],[-Math.PI/2,0,0]],[new n(u,c),null,[Math.PI/2,0,0]]],XYZ:[[new n(new U(.1,0),D.clone()),[0,0,0]]],XY:[[new n(new m(.15,.15,.01),T.clone()),[.15,.15,0]]],YZ:[[new n(new m(.15,.15,.01),y.clone()),[0,.15,.15],[0,Math.PI/2,0]]],XZ:[[new n(new m(.15,.15,.01),X.clone()),[.15,0,.15],[-Math.PI/2,0,0]]]},ft={X:[[new n(new w(.2,0,.6,4),o),[.3,0,0],[0,0,-Math.PI/2]],[new n(new w(.2,0,.6,4),o),[-.3,0,0],[0,0,Math.PI/2]]],Y:[[new n(new w(.2,0,.6,4),o),[0,.3,0]],[new n(new w(.2,0,.6,4),o),[0,-.3,0],[0,0,Math.PI]]],Z:[[new n(new w(.2,0,.6,4),o),[0,0,.3],[Math.PI/2,0,0]],[new n(new w(.2,0,.6,4),o),[0,0,-.3],[-Math.PI/2,0,0]]],XYZ:[[new n(new U(.2,0),o)]],XY:[[new n(new m(.2,.2,.01),o),[.15,.15,0]]],YZ:[[new n(new m(.2,.2,.01),o),[0,.15,.15],[0,Math.PI/2,0]]],XZ:[[new n(new m(.2,.2,.01),o),[.15,0,.15],[-Math.PI/2,0,0]]]},wt={START:[[new n(new U(.01,2),e),null,null,null,"helper"]],END:[[new n(new U(.01,2),e),null,null,null,"helper"]],DELTA:[[new I(H(),e),null,null,null,"helper"]],X:[[new I(S,e.clone()),[-1e3,0,0],null,[1e6,1,1],"helper"]],Y:[[new I(S,e.clone()),[0,-1e3,0],[0,0,Math.PI/2],[1e6,1,1],"helper"]],Z:[[new I(S,e.clone()),[0,0,-1e3],[0,-Math.PI/2,0],[1e6,1,1],"helper"]]},yt={XYZE:[[new n(b(.5,1),C),null,[0,Math.PI/2,0]]],X:[[new n(b(.5,.5),l)]],Y:[[new n(b(.5,.5),t),null,[0,0,-Math.PI/2]]],Z:[[new n(b(.5,.5),c),null,[0,Math.PI/2,0]]],E:[[new n(b(.75,1),j),null,[0,Math.PI/2,0]]]},_t={AXIS:[[new I(S,e.clone()),[-1e3,0,0],null,[1e6,1,1],"helper"]]},bt={XYZE:[[new n(new Et(.25,10,8),o)]],X:[[new n(new q(.5,.1,4,24),o),[0,0,0],[0,-Math.PI/2,-Math.PI/2]]],Y:[[new n(new q(.5,.1,4,24),o),[0,0,0],[Math.PI/2,0,0]]],Z:[[new n(new q(.5,.1,4,24),o),[0,0,0],[0,0,-Math.PI/2]]],E:[[new n(new q(.75,.1,2,24),o)]]},Pt={X:[[new n(v,l),[.5,0,0],[0,0,-Math.PI/2]],[new n(u,l),[0,0,0],[0,0,-Math.PI/2]],[new n(v,l),[-.5,0,0],[0,0,Math.PI/2]]],Y:[[new n(v,t),[0,.5,0]],[new n(u,t)],[new n(v,t),[0,-.5,0],[0,0,Math.PI]]],Z:[[new n(v,c),[0,0,.5],[Math.PI/2,0,0]],[new n(u,c),[0,0,0],[Math.PI/2,0,0]],[new n(v,c),[0,0,-.5],[-Math.PI/2,0,0]]],XY:[[new n(new m(.15,.15,.01),T),[.15,.15,0]]],YZ:[[new n(new m(.15,.15,.01),y),[0,.15,.15],[0,Math.PI/2,0]]],XZ:[[new n(new m(.15,.15,.01),X),[.15,0,.15],[-Math.PI/2,0,0]]],XYZ:[[new n(new m(.1,.1,.1),D.clone())]]},St={X:[[new n(new w(.2,0,.6,4),o),[.3,0,0],[0,0,-Math.PI/2]],[new n(new w(.2,0,.6,4),o),[-.3,0,0],[0,0,Math.PI/2]]],Y:[[new n(new w(.2,0,.6,4),o),[0,.3,0]],[new n(new w(.2,0,.6,4),o),[0,-.3,0],[0,0,Math.PI]]],Z:[[new n(new w(.2,0,.6,4),o),[0,0,.3],[Math.PI/2,0,0]],[new n(new w(.2,0,.6,4),o),[0,0,-.3],[-Math.PI/2,0,0]]],XY:[[new n(new m(.2,.2,.01),o),[.15,.15,0]]],YZ:[[new n(new m(.2,.2,.01),o),[0,.15,.15],[0,Math.PI/2,0]]],XZ:[[new n(new m(.2,.2,.01),o),[.15,0,.15],[-Math.PI/2,0,0]]],XYZ:[[new n(new m(.2,.2,.2),o),[0,0,0]]]},xt={X:[[new I(S,e.clone()),[-1e3,0,0],null,[1e6,1,1],"helper"]],Y:[[new I(S,e.clone()),[0,-1e3,0],[0,0,Math.PI/2],[1e6,1,1],"helper"]],Z:[[new I(S,e.clone()),[0,0,-1e3],[0,-Math.PI/2,0],[1e6,1,1],"helper"]]};function E(_){const W=new et;for(const P in _)for(let z=_[P].length;z--;){const f=_[P][z][0].clone(),G=_[P][z][1],F=_[P][z][2],B=_[P][z][3],gt=_[P][z][4];f.name=P,f.tag=gt,G&&f.position.set(G[0],G[1],G[2]),F&&f.rotation.set(F[0],F[1],F[2]),B&&f.scale.set(B[0],B[1],B[2]),f.updateMatrix();const nt=f.geometry.clone();nt.applyMatrix4(f.matrix),f.geometry=nt,f.renderOrder=1/0,f.position.set(0,0,0),f.rotation.set(0,0,0),f.scale.set(1,1,1),W.add(f)}return W}this.gizmo={},this.picker={},this.helper={},this.add(this.gizmo.translate=E(Y)),this.add(this.gizmo.rotate=E(yt)),this.add(this.gizmo.scale=E(Pt)),this.add(this.picker.translate=E(ft)),this.add(this.picker.rotate=E(bt)),this.add(this.picker.scale=E(St)),this.add(this.helper.translate=E(wt)),this.add(this.helper.rotate=E(_t)),this.add(this.helper.scale=E(xt)),this.picker.translate.visible=!1,this.picker.rotate.visible=!1,this.picker.scale.visible=!1}updateMatrixWorld(i){const o=(this.mode==="scale"?"local":this.space)==="local"?this.worldQuaternion:K;this.gizmo.translate.visible=this.mode==="translate",this.gizmo.rotate.visible=this.mode==="rotate",this.gizmo.scale.visible=this.mode==="scale",this.helper.translate.visible=this.mode==="translate",this.helper.rotate.visible=this.mode==="rotate",this.helper.scale.visible=this.mode==="scale";let e=[];e=e.concat(this.picker[this.mode].children),e=e.concat(this.gizmo[this.mode].children),e=e.concat(this.helper[this.mode].children);for(let l=0;l<e.length;l++){const t=e[l];t.visible=!0,t.rotation.set(0,0,0),t.position.copy(this.worldPosition);let c;if(this.camera.isOrthographicCamera?c=(this.camera.top-this.camera.bottom)/this.camera.zoom:c=this.worldPosition.distanceTo(this.cameraPosition)*Math.min(1.9*Math.tan(Math.PI*this.camera.fov/360)/this.camera.zoom,7),t.scale.set(1,1,1).multiplyScalar(c*this.size/4),t.tag==="helper"){t.visible=!1,t.name==="AXIS"?(t.visible=!!this.axis,this.axis==="X"&&(h.setFromEuler(N.set(0,0,0)),t.quaternion.copy(o).multiply(h),Math.abs(r.copy(O).applyQuaternion(o).dot(this.eye))>.9&&(t.visible=!1)),this.axis==="Y"&&(h.setFromEuler(N.set(0,0,Math.PI/2)),t.quaternion.copy(o).multiply(h),Math.abs(r.copy(Z).applyQuaternion(o).dot(this.eye))>.9&&(t.visible=!1)),this.axis==="Z"&&(h.setFromEuler(N.set(0,Math.PI/2,0)),t.quaternion.copy(o).multiply(h),Math.abs(r.copy(R).applyQuaternion(o).dot(this.eye))>.9&&(t.visible=!1)),this.axis==="XYZE"&&(h.setFromEuler(N.set(0,Math.PI/2,0)),r.copy(this.rotationAxis),t.quaternion.setFromRotationMatrix(pt.lookAt(ct,r,Z)),t.quaternion.multiply(h),t.visible=this.dragging),this.axis==="E"&&(t.visible=!1)):t.name==="START"?(t.position.copy(this.worldPositionStart),t.visible=this.dragging):t.name==="END"?(t.position.copy(this.worldPosition),t.visible=this.dragging):t.name==="DELTA"?(t.position.copy(this.worldPositionStart),t.quaternion.copy(this.worldQuaternionStart),d.set(1e-10,1e-10,1e-10).add(this.worldPositionStart).sub(this.worldPosition).multiplyScalar(-1),d.applyQuaternion(this.worldQuaternionStart.clone().invert()),t.scale.copy(d),t.visible=this.dragging):(t.quaternion.copy(o),this.dragging?t.position.copy(this.worldPositionStart):t.position.copy(this.worldPosition),this.axis&&(t.visible=this.axis.search(t.name)!==-1));continue}t.quaternion.copy(o),this.mode==="translate"||this.mode==="scale"?(t.name==="X"&&Math.abs(r.copy(O).applyQuaternion(o).dot(this.eye))>.99&&(t.scale.set(1e-10,1e-10,1e-10),t.visible=!1),t.name==="Y"&&Math.abs(r.copy(Z).applyQuaternion(o).dot(this.eye))>.99&&(t.scale.set(1e-10,1e-10,1e-10),t.visible=!1),t.name==="Z"&&Math.abs(r.copy(R).applyQuaternion(o).dot(this.eye))>.99&&(t.scale.set(1e-10,1e-10,1e-10),t.visible=!1),t.name==="XY"&&Math.abs(r.copy(R).applyQuaternion(o).dot(this.eye))<.2&&(t.scale.set(1e-10,1e-10,1e-10),t.visible=!1),t.name==="YZ"&&Math.abs(r.copy(O).applyQuaternion(o).dot(this.eye))<.2&&(t.scale.set(1e-10,1e-10,1e-10),t.visible=!1),t.name==="XZ"&&Math.abs(r.copy(Z).applyQuaternion(o).dot(this.eye))<.2&&(t.scale.set(1e-10,1e-10,1e-10),t.visible=!1)):this.mode==="rotate"&&(V.copy(o),r.copy(this.eye).applyQuaternion(h.copy(o).invert()),t.name.search("E")!==-1&&t.quaternion.setFromRotationMatrix(pt.lookAt(this.eye,ct,Z)),t.name==="X"&&(h.setFromAxisAngle(O,Math.atan2(-r.y,r.z)),h.multiplyQuaternions(V,h),t.quaternion.copy(h)),t.name==="Y"&&(h.setFromAxisAngle(Z,Math.atan2(r.x,r.z)),h.multiplyQuaternions(V,h),t.quaternion.copy(h)),t.name==="Z"&&(h.setFromAxisAngle(R,Math.atan2(r.y,r.x)),h.multiplyQuaternions(V,h),t.quaternion.copy(h))),t.visible=t.visible&&(t.name.indexOf("X")===-1||this.showX),t.visible=t.visible&&(t.name.indexOf("Y")===-1||this.showY),t.visible=t.visible&&(t.name.indexOf("Z")===-1||this.showZ),t.visible=t.visible&&(t.name.indexOf("E")===-1||this.showX&&this.showY&&this.showZ),t.material._color=t.material._color||t.material.color.clone(),t.material._opacity=t.material._opacity||t.material.opacity,t.material.color.copy(t.material._color),t.material.opacity=t.material._opacity,this.enabled&&this.axis&&(t.name===this.axis||this.axis.split("").some(function(y){return t.name===y}))&&(t.material.color.setHex(16776960),t.material.opacity=1)}super.updateMatrixWorld(i)}}class Dt extends n{constructor(){super(new It(1e5,1e5,2,2),new mt({visible:!1,wireframe:!0,side:Qt,transparent:!0,opacity:.1,toneMapped:!1})),this.isTransformControlsPlane=!0,this.type="TransformControlsPlane"}updateMatrixWorld(i){let s=this.space;switch(this.position.copy(this.worldPosition),this.mode==="scale"&&(s="local"),J.copy(O).applyQuaternion(s==="local"?this.worldQuaternion:K),k.copy(Z).applyQuaternion(s==="local"?this.worldQuaternion:K),L.copy(R).applyQuaternion(s==="local"?this.worldQuaternion:K),r.copy(k),this.mode){case"translate":case"scale":switch(this.axis){case"X":r.copy(this.eye).cross(J),x.copy(J).cross(r);break;case"Y":r.copy(this.eye).cross(k),x.copy(k).cross(r);break;case"Z":r.copy(this.eye).cross(L),x.copy(L).cross(r);break;case"XY":x.copy(L);break;case"YZ":x.copy(J);break;case"XZ":r.copy(L),x.copy(k);break;case"XYZ":case"E":x.set(0,0,0);break}break;case"rotate":default:x.set(0,0,0)}x.length()===0?this.quaternion.copy(this.cameraQuaternion):(dt.lookAt(d.set(0,0,0),x,r),this.quaternion.setFromRotationMatrix(dt)),super.updateMatrixWorld(i)}}export{qt as T};
