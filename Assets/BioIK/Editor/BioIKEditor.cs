﻿using BioIK.Helpers;
using BioIK.Setup;
using BioIK.Setup.Objectives;
using UnityEditor;
using UnityEngine;

namespace BioIK.Editor {
	[CustomEditor(typeof(BioIK))]
	public class BioIKEditor : UnityEditor.Editor {

		public BioIK Target;
		public Transform TargetTransform;

		private Color Color1 = Color.white;
		//private Color Color2 = Color.black;
		private Color Color3 = new(0.6f, 0.6f, 0.6f, 1f);
		private Color Color4 = new(0.3f, 0.8f, 0.8f, 1f);
		private Color Color5 = new(1f, 0.7f, 0.3f, 1f);
		private Color Color6 = new(0.9f, 0.3f, 0.9f, 1f);
		private Color Color7 = new(1.0f, 0.3f, 0.3f, 1f);
		private Color Color8 = new(0.3f, 0.6f, 0.6f, 1f);
		private Color Color9 = new(0.4f, 0.9f, 0.4f, 1f);	
		//private Color Color10 = new Color(1f, 0.5f, 1f, 1f);
		private Color Color11 = new(0.3f, 0.3f, 0.3f, 1f);
		//private Color Color12 = new Color(0.9f, 0.6f, 0.9f, 1f);
		private Color Color13 = new(0.5f, 0.5f, 0.5f, 1f);
		private Color Color14 = new(0.75f, 0.75f, 0.75f, 1f);

		private bool ChosingObjectiveType;

		private bool IsPlaying;
		private bool IsEnabled;

		private void Awake() {
			EditorApplication.playModeStateChanged += PlaymodeStateChanged;
			Target = (BioIK)target;
			TargetTransform = Target.transform;
			Target.Refresh(false);
		}

		private void OnEnable() {
			IsEnabled = true;
		}

		private void OnDisable() {
			IsEnabled = false;
		}

		private void MakeVisible(Transform t) {
			if(t.GetComponent<BioSegment>()) {
				t.GetComponent<BioSegment>().hideFlags = HideFlags.None;
			}
			foreach(BioObjective o in t.GetComponents<BioObjective>()) {
				o.hideFlags = HideFlags.None;
			}
			if(t.GetComponent<BioJoint>()) {
				t.GetComponent<BioJoint>().hideFlags = HideFlags.None;
			}
			for(int i=0; i<t.childCount; i++) {
				MakeVisible(t.GetChild(i));
			}
		}

		private void MakeInvisible(Transform t) {
			if(t.GetComponent<BioSegment>()) {
				t.GetComponent<BioSegment>().hideFlags = HideFlags.HideInInspector;
			}
			foreach(BioObjective o in t.GetComponents<BioObjective>()) {
				o.hideFlags = HideFlags.HideInInspector;
			}
			if(t.GetComponent<BioJoint>()) {
				t.GetComponent<BioJoint>().hideFlags = HideFlags.HideInInspector;
			}
			for(int i=0; i<t.childCount; i++) {
				MakeInvisible(t.GetChild(i));
			}
		}

		private void OnDestroy() {
			if(
				Target == null 
				&& 
				TargetTransform != null 
				&& 
				!IsPlaying 
				&& 
				!IsEnabled
				&&
				!EditorApplication.isPlayingOrWillChangePlaymode
				) {
				Utility.Cleanup(TargetTransform);
			}
		}

		private void PlaymodeStateChanged(PlayModeStateChange state) {
			IsPlaying = Application.isPlaying;
		}

		public override void OnInspectorGUI() {
			Undo.RecordObject(Target, Target.name);

			SetGUIColor(Color3);
			using(new EditorGUILayout.VerticalScope ("Button")) {
				SetGUIColor(Color5);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				EditorGUILayout.HelpBox("                    Settings                    ", MessageType.None);
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();

				SetGUIColor(Color1);
				Target.generations = EditorGUILayout.IntField("Generations", Target.generations);
				SetGUIColor(Color1);
				Target.SetPopulationSize(EditorGUILayout.IntField("Individuals", Target.PopulationSize));
				SetGUIColor(Color1);
				Target.SetElites(EditorGUILayout.IntField("Elites", Target.Elites));
				SetGUIColor(Color1);
			}

			SetGUIColor(Color3);
			using(new EditorGUILayout.VerticalScope ("Button")) {
				SetGUIColor(Color5);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				EditorGUILayout.HelpBox("                   Character                   ", MessageType.None);
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();

				//SetGUIColor(Color5);
				//EditorGUILayout.HelpBox("Degree of Freedom: " + new Model(Target).GetDoF() + " / " + DoF, MessageType.None);

				int maxIndent = 0;
				ComputeMaxIndentLevel(Target.transform, 0, ref maxIndent);

				Target.scroll = EditorGUILayout.BeginScrollView(Target.scroll, GUILayout.Height(500f));
				InspectBody(Target.FindSegment(Target.transform), 0, maxIndent);
				EditorGUILayout.EndScrollView();
			}

			SetGUIColor(Color1);
			if(Target != null) {
				EditorUtility.SetDirty(Target);
			}
		}

		private void InspectBody(BioSegment segment, int indent, int maxIndent) {
			SetGUIColor(Color11);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				EditorGUILayout.BeginHorizontal();
				
				if(Target.selectedSegment != segment) {
					if(indent > 0) {
						SetGUIColor(Color13);
						using(new EditorGUILayout.VerticalScope ("Box")) {
							int width = 10*(indent-1);
							EditorGUILayout.LabelField("", GUILayout.Width(width));
						}
					}
				}

				GUI.skin.button.alignment = TextAnchor.MiddleLeft;
				if(segment == Target.selectedSegment) {
					SetGUIColor(Color5);
				} else {
					SetGUIColor(Color.Lerp(Color4, Color8, (float)indent / (float)maxIndent));
				}
				if(GUILayout.Button(segment.transform.name, GUILayout.Height(25f), GUILayout.ExpandWidth(true))) {
					if(Target.selectedSegment == segment) {
						Target.selectedSegment = null;
						ChosingObjectiveType = false;
					} else {
						Target.selectedSegment = segment;
					}
				}

				EditorGUILayout.EndHorizontal();

				if(Target.selectedSegment == segment) {
					InspectSegment(segment);
				} else {
					GUILayout.BeginHorizontal();
					GUILayout.FlexibleSpace();
					if(segment.joint != null) {
						SetGUIColor(Color6);
						GUILayout.Box(" Joint ");
					}
					foreach(BioObjective objective in segment.objectives) {
						SetGUIColor(Color9);
						GUILayout.Box(" " + objective.GetObjectiveType().ToString() + " ");
					}
					GUILayout.FlexibleSpace();
					GUILayout.EndHorizontal();
				}
			}
			
			for(int i=0; i<segment.children.Length; i++) {
				InspectBody(segment.children[i], indent+1, maxIndent);
			}
		}

		private void InspectSegment(BioSegment segment) {
			Undo.RecordObject(segment, segment.name);

			SetGUIColor(Color13);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				
				SetGUIColor(Color4);
				Vector3 A = segment.parent == null ? segment.GetAnchoredPosition() : segment.parent.GetAnchoredPosition();
				Vector3 B = segment.GetAnchoredPosition();
				EditorGUILayout.HelpBox("Link Length: " + Vector3.Distance(A,B), MessageType.None);

				SetGUIColor(Color6);
				using(new EditorGUILayout.VerticalScope ("Box")) {
					if(segment.joint == null) {
						GUI.skin.button.alignment = TextAnchor.MiddleCenter;
						SetGUIColor(Color1);
						if(GUILayout.Button("Add Joint")) {
							segment.AddJoint();
						}
					} else {
						InspectJoint(segment.joint);
					}
				}

			}

			for(int i=0; i<segment.objectives.Length; i++) {
				SetGUIColor(Color13);
				using(new EditorGUILayout.VerticalScope ("Box")) {

					SetGUIColor(Color9);
					using(new EditorGUILayout.VerticalScope ("Box")) {
						InspectObjective(segment.objectives[i]);
					}

				}
			}

			SetGUIColor(Color13);
			using(new EditorGUILayout.VerticalScope ("Box")) {

				SetGUIColor(Color9);
				using(new EditorGUILayout.VerticalScope ("Box")) {

					GUI.skin.button.alignment = TextAnchor.MiddleCenter;
					SetGUIColor(ChosingObjectiveType ? Color14 : Color1);
					if(GUILayout.Button("Add Objective")) {
						ChosingObjectiveType = !ChosingObjectiveType;
					}
					if(ChosingObjectiveType) {
						SetGUIColor(Color8);
						using(new EditorGUILayout.VerticalScope ("Box")) {
							int count = System.Enum.GetValues(typeof(ObjectiveType)).Length;
							string[] names = System.Enum.GetNames(typeof(ObjectiveType));
							for(int i=0; i<count; i++) {
								SetGUIColor(Color1);
								if(GUILayout.Button(names[i])) {
									ChosingObjectiveType = false;
									segment.AddObjective((ObjectiveType)i);
								}
							}
						}
					}
				}

			}
			
			if(segment != null) {
				EditorUtility.SetDirty(segment);
			}
		}

		private void InspectJoint(BioJoint joint) {
			Undo.RecordObject(joint, joint.name);

			SetGUIColor(Color13);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				SetGUIColor(Color5);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				EditorGUILayout.HelpBox("                    Joint                    ", MessageType.None);
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();

				SetGUIColor(Color1);
				joint.enabled = EditorGUILayout.Toggle("Enabled", joint.enabled);

				SetGUIColor(Color4);
				EditorGUILayout.HelpBox("Geometry", MessageType.None);
				SetGUIColor(Color1);
				joint.jointType = (JointType)EditorGUILayout.EnumPopup("Joint Type", joint.jointType);
				SetGUIColor(Color1);
				joint.SetOrientation(EditorGUILayout.Vector3Field("Orientation", joint.GetOrientation()));
				SetGUIColor(Color4);
				EditorGUILayout.HelpBox("Default Frame", MessageType.None);
				SetGUIColor(Color1);
				Vector3 defaultPosition = EditorGUILayout.Vector3Field("Position", joint.GetDefaultPosition());
				SetGUIColor(Color1);
				Quaternion defaultRotation = Quaternion.Euler(EditorGUILayout.Vector3Field("Rotation", joint.GetDefaultRotation().eulerAngles));
				joint.SetDefaultFrame(defaultPosition, defaultRotation);

				InspectMotion(joint.x, "     X Motion     ");
				InspectMotion(joint.y, "     Y Motion     ");
				InspectMotion(joint.z, "     Z Motion     ");

				GUI.skin.button.alignment = TextAnchor.MiddleCenter;
				SetGUIColor(Color7);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				if(GUILayout.Button("Remove", GUILayout.Width(100f))) {
					joint.Remove();
				}
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();
			}

			if(joint != null) {
				joint.UpdateData();
				joint.ProcessMotion();
				EditorUtility.SetDirty(joint);
			}
		}

		private void InspectMotion(BioJoint.Motion motion, string name) {
			SetGUIColor(Color8);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				SetGUIColor(Color5);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				EditorGUILayout.HelpBox(name, MessageType.None);
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();

				if(motion.IsEnabled()) {
					SetGUIColor(Color1);
					motion.constrained = EditorGUILayout.Toggle("Constrained", motion.constrained);
					if(motion.constrained) {
						SetGUIColor(Color1);
						motion.SetLowerLimit(EditorGUILayout.DoubleField("Lower Limit", motion.GetLowerLimit()));
						SetGUIColor(Color1);
						motion.SetUpperLimit(EditorGUILayout.DoubleField("Upper Limit", motion.GetUpperLimit()));
						SetGUIColor(Color1);
						motion.SetTargetValue(EditorGUILayout.Slider("Target Value", (float)motion.GetTargetValue(), (float)motion.GetLowerLimit(), (float)motion.GetUpperLimit()));
					} else {
						SetGUIColor(Color1);
						motion.SetTargetValue(EditorGUILayout.DoubleField("Target Value", motion.GetTargetValue()));
					}
					
					GUI.skin.button.alignment = TextAnchor.MiddleCenter;
					SetGUIColor(Color1);
					GUILayout.BeginHorizontal();
					GUILayout.FlexibleSpace();
					if(GUILayout.Button("Disable", GUILayout.Width(250f), GUILayout.Height(20f))) {
						motion.SetEnabled(false);
					}
					GUILayout.FlexibleSpace();
					GUILayout.EndHorizontal();
				} else {
					GUI.skin.button.alignment = TextAnchor.MiddleCenter;
					SetGUIColor(Color1);
					GUILayout.BeginHorizontal();
					GUILayout.FlexibleSpace();
					if(GUILayout.Button("Enable", GUILayout.Width(250f), GUILayout.Height(20f))) {
						motion.SetEnabled(true);
					}
					GUILayout.FlexibleSpace();
					GUILayout.EndHorizontal();
				}
			}
		}

		private void InspectObjective(BioObjective objective) {
			Undo.RecordObject(objective, objective.name);

			SetGUIColor(Color13);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				SetGUIColor(Color5);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				EditorGUILayout.HelpBox("                    Objective (" + objective.GetObjectiveType().ToString() + ")                    ", MessageType.None);
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();
				
				SetGUIColor(Color1);
				objective.enabled = EditorGUILayout.Toggle("Enabled", objective.enabled);

				SetGUIColor(Color1);
				objective.Weight = EditorGUILayout.DoubleField("Weight", objective.Weight);

				switch(objective.GetObjectiveType()) {
					case ObjectiveType.Position:
					InspectPosition((Position)objective);
					break;

					case ObjectiveType.Orientation:
					InspectOrientation((Orientation)objective);
					break;
				}

				GUI.skin.button.alignment = TextAnchor.MiddleCenter;
				SetGUIColor(Color7);
				GUILayout.BeginHorizontal();
				GUILayout.FlexibleSpace();
				if(GUILayout.Button("Remove", GUILayout.Width(100f))) {
					objective.Remove();
				}
				GUILayout.FlexibleSpace();
				GUILayout.EndHorizontal();
			}

			if(objective != null) {
				EditorUtility.SetDirty(objective);
			}
		}

		private void InspectPosition(Position objective) {
			SetGUIColor(Color1);
			objective.SetTargetTransform(EditorGUILayout.ObjectField("Target Transform", objective.GetTargetTransform(), typeof(Transform), true) as Transform);
			SetGUIColor(Color1);
			objective.SetTargetPosition(EditorGUILayout.Vector3Field("Target Position", objective.GetTargetPosition()));
			SetGUIColor(Color1);
			objective.SetMaximumError(EditorGUILayout.DoubleField("Maximum Error", objective.GetMaximumError()));
		}

		private void InspectOrientation(Orientation objective) {
			SetGUIColor(Color1);
			objective.SetTargetTransform(EditorGUILayout.ObjectField("Target Transform", objective.GetTargetTransform(), typeof(Transform), true) as Transform);
			SetGUIColor(Color1);
			objective.SetTargetRotation(EditorGUILayout.Vector3Field("Target Rotation", objective.GetTargetRotattion()));
			SetGUIColor(Color1);
			objective.SetMaximumError(EditorGUILayout.DoubleField("Maximum Error", objective.GetMaximumError()));
		}

		public void OnSceneGUI() {
			DrawSkeleton(Target.FindSegment(Target.transform));
			DrawSetup(Target.FindSegment(Target.transform), false);
			if(Target.selectedSegment != null) {
				DrawSegment(Target.selectedSegment, true);
			}
		}

		private bool StopDraw(BioSegment segment) {
			if(segment.parent == null) {
				return false;
			}
			return false;
			//return segment.Parent.name == "Kopf" && segment.name != "Head" && segment.name != "Ohr_R" && segment.name != "Ohr_L" || segment.name == "Ohr_L_end" || segment.name == "Ohr_R_end";
		}

		private void DrawSkeleton(BioSegment segment) {
			if(!StopDraw(segment)) {
			if(segment.parent != null) {
				DrawLine(segment.parent.GetAnchoredPosition(), segment.GetAnchoredPosition(), 5f, Color.cyan);
			}
			for(int i=0; i<segment.children.Length; i++) {
				DrawSkeleton(segment.children[i]);
			}
			}
		}

		private void DrawSetup(BioSegment segment, bool final) {
			if(!StopDraw(segment)) {
			DrawSegment(segment, final);
			for(int i=0; i<segment.children.Length; i++) {
				DrawSetup(segment.children[i], final);
			}
			}
		}

		private void DrawSegment(BioSegment segment, bool final) {
			Vector3 P = segment.GetAnchoredPosition();
			if(Target.selectedSegment == segment && final) {
				DrawSphere(P, 0.25f, new(0f, 0f, 0f, 0.4f));
				DrawSphere(P, 0.02f, Color5);
			} else if(Target.selectedSegment != segment) {
				DrawSphere(P, 0.02f, Color.cyan);
			}
			if(segment.joint != null) {
				DrawJoint(segment.joint, final);
			}
			for(int i=0; i<segment.objectives.Length; i++) {
				DrawObjective(segment.objectives[i], final);
			}
		}

		private void DrawJoint(BioJoint joint, bool final) {
			if(!final) {
				joint.GetDoF();
			}
			//DrawDottedLine(joint.Segment.Transform.position, joint.GetAnchorInWorldSpace(), 5f, Color.magenta);
			DrawCube(joint.GetAnchorInWorldSpace(), joint.segment.transform.rotation * Quaternion.Euler(joint.GetOrientation()), 0.025f, Color.magenta);
			DrawMotion(joint.x, Color.red, final);
			DrawMotion(joint.y, Color.green, final);
			DrawMotion(joint.z, Color.blue, final);
		}

		private void DrawMotion(BioJoint.Motion motion, Color color, bool final) {
			if(Target.selectedSegment == motion.joint.segment && final) {
				DrawArrow(motion.joint.GetAnchorInWorldSpace(), motion.joint.segment.transform.rotation * Quaternion.LookRotation(motion.axis), 0.125f, motion.IsEnabled() ? color : Color.grey);
				
				if(!motion.IsEnabled() || !motion.constrained) {
					return;
				}

				if(motion.joint.jointType == JointType.Rotational) {
					if(motion == motion.joint.x)
					{
						var rotation = motion.joint.segment.transform.rotation;
						DrawSolidArc(
							motion.joint.GetAnchorInWorldSpace(), 
							rotation * motion.axis, 
							Quaternion.AngleAxis((float)motion.GetLowerLimit(), rotation * motion.axis) * rotation * motion.joint.y.axis, 
							(float)motion.GetUpperLimit() - (float)motion.GetLowerLimit(), 
							0.125f, 
							new(1f, 0f, 0f, 0.25f)
							);
					}
					if(motion == motion.joint.y)
					{
						var rotation = motion.joint.segment.transform.rotation;
						DrawSolidArc(
							motion.joint.GetAnchorInWorldSpace(), 
							rotation * motion.axis, 
							Quaternion.AngleAxis((float)motion.GetLowerLimit(), rotation * motion.axis) * rotation * motion.joint.z.axis, 
							(float)motion.GetUpperLimit() - (float)motion.GetLowerLimit(), 
							0.125f, 
							new(0f, 1f, 0f, 0.25f)
							);
					}
					if(motion == motion.joint.z)
					{
						var rotation = motion.joint.segment.transform.rotation;
						DrawSolidArc(
							motion.joint.GetAnchorInWorldSpace(), 
							rotation * motion.axis, 
							Quaternion.AngleAxis((float)motion.GetLowerLimit(), rotation * motion.axis) * rotation * motion.joint.x.axis, 
							(float)motion.GetUpperLimit() - (float)motion.GetLowerLimit(), 
							0.125f, 
							new(0f, 0f, 1f, 0.25f)
							);
					}
				}

				if(motion.joint.jointType == JointType.Translational) {
					Quaternion rotation = motion.joint.segment.transform.rotation;
					Vector3 A = motion.joint.GetAnchorInWorldSpace() + (float)motion.GetLowerLimit() * (rotation * motion.axis);
					Vector3 B = motion.joint.GetAnchorInWorldSpace() + (float)motion.GetUpperLimit() * (rotation * motion.axis);
					Color c = Color.white;
					if(motion == motion.joint.x) {
						c = Color.red;
					}
					if(motion == motion.joint.y) {
						c = Color.green;
					}
					if(motion == motion.joint.z) {
						c = Color.blue;
					}
					DrawLine(A,	B, 3f, c);
					rotation = motion.joint.segment.transform.rotation;
					DrawCube(A, rotation, 0.0125f, new(c.r, c.g, c.b, 0.5f));
					DrawCube(B, rotation, 0.0125f, new(c.r, c.g, c.b, 0.5f));
				}

			} else if(Target.selectedSegment != motion.joint.segment) {
				DrawArrow(motion.joint.GetAnchorInWorldSpace(), motion.joint.segment.transform.rotation * Quaternion.LookRotation(motion.axis), 0.05f, motion.IsEnabled() ? color : Color.clear);
			}
		}

		private void DrawObjective(BioObjective objective, bool selected) {
			if(!selected) {
				return;
			}

			switch(objective.GetObjectiveType()) {
				case ObjectiveType.Position:
				DrawPosition((Position)objective);
				break;

				case ObjectiveType.Orientation:
				DrawOrientation((Orientation)objective);
				break;
			}
		}

		private void DrawPosition(Position objective) {
			DrawSphere(objective.GetTargetPosition(), 0.1f, new(1f, 0f, 0f, 0.75f));
			Handles.Label(objective.GetTargetPosition(), "Target");
		}

		private void DrawOrientation(Orientation objective) {
			Quaternion rotation = Quaternion.Euler(objective.GetTargetRotattion());
			Vector3 right = rotation * Vector3.right;
			Vector3 up = rotation * Vector3.up;
			Vector3 forward = rotation * Vector3.forward;
			float length = 0.1f;
			Vector3 position = objective.Segment.transform.position;
			DrawLine(position - length * right, position + length * right, 5f, new(1f, 0f, 0f, 0.75f));
			DrawLine(position - length * up, position + length * up, 5f, new(0f, 1f, 0f, 0.75f));
			DrawLine(position - length * forward, position + length * forward, 5f, new(0f, 0f, 1f, 0.75f));
			Handles.Label(position, "Target");
		}

		private void DrawSphere(Vector3 position, float radius, Color color) {
			Handles.color = color;
			Handles.SphereHandleCap(0, position, Quaternion.identity, radius, EventType.Repaint);
		}

		private void DrawCube(Vector3 position, Quaternion rotation, float size, Color color) {
			Handles.color = color;
			Handles.CubeHandleCap(0, position, rotation, size, EventType.Repaint);
		}

		private void DrawLine(Vector3 a, Vector3 b, float width, Color color) {
			Handles.color = color;
			Handles.DrawAAPolyLine(width, new Vector3[2] {a,b});
		}

		private void DrawArrow(Vector3 position, Quaternion rotation, float length, Color color) {
			Handles.color = color;
			Handles.ArrowHandleCap(0, position, rotation, length, EventType.Repaint);
		}

		private void DrawSolidArc(Vector3 position, Vector3 normal, Vector3 from, float angle, float radius, Color color) {
			Handles.color = color;
			Handles.DrawSolidArc(position, normal, from, angle, radius);
		}

		private void SetGUIColor(Color color) {
			GUI.backgroundColor = color;
		}

		private void ComputeMaxIndentLevel(Transform t, int level, ref int maxIndent) {
			maxIndent = System.Math.Max(maxIndent, level);
			for(int i=0; i<t.childCount; i++) {
				ComputeMaxIndentLevel(t.GetChild(i), level+1, ref maxIndent);
			}
		}

	}
}
