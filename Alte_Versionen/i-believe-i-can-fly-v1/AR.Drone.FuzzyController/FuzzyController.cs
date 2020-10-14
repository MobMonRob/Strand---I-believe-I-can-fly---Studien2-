using AForge.Fuzzy;

namespace Fuzzy
{	
    public class FuzzyController
    {	
        private InferenceSystem infSystem;

	    public void InitFIS()
	    {
            // Definition for Variable: backward
			LinguisticVariable lvBackward = new LinguisticVariable("backward", -1.00f, 1.00f );
			FuzzySet fsStrongforward = new FuzzySet("strongForward", new TrapezoidalFunction( -0.80f, -0.40f,  TrapezoidalFunction.EdgeType.Right ));
			FuzzySet fsMediumforward = new FuzzySet("mediumForward", new TrapezoidalFunction( -0.90f, -0.60f, -0.40f, 0.00f ));
			FuzzySet fsMediumbackward = new FuzzySet("mediumBackward", new TrapezoidalFunction( 0.00f, 0.40f, 0.60f, 0.90f ));
			FuzzySet fsStrongbackward = new FuzzySet("strongBackward", new TrapezoidalFunction( 0.40f, 0.80f,  TrapezoidalFunction.EdgeType.Left ));
			FuzzySet fsZero = new FuzzySet("zero", new TrapezoidalFunction( -0.60f, -0.20f, 0.20f, 0.60f ));
			
			lvBackward.AddLabel(fsStrongforward);
			lvBackward.AddLabel(fsMediumforward);
			lvBackward.AddLabel(fsMediumbackward);
			lvBackward.AddLabel(fsStrongbackward);
			lvBackward.AddLabel(fsZero);
			
			
			// Definition for Variable: sideward
			LinguisticVariable lvSideward = new LinguisticVariable("sideward", -1.00f, 1.00f );
			FuzzySet fsStrongleft = new FuzzySet("strongLeft", new TrapezoidalFunction( -0.80f, -0.40f,  TrapezoidalFunction.EdgeType.Right ));
			FuzzySet fsMediumleft = new FuzzySet("mediumLeft", new TrapezoidalFunction( -0.90f, -0.60f, -0.40f, 0.00f ));
			FuzzySet fsMediumright = new FuzzySet("mediumRight", new TrapezoidalFunction( 0.00f, 0.40f, 0.60f, 0.90f ));
			FuzzySet fsStrongright = new FuzzySet("strongRight", new TrapezoidalFunction( 0.40f, 0.80f,  TrapezoidalFunction.EdgeType.Left ));
			FuzzySet fsZero0 = new FuzzySet("zero", new TrapezoidalFunction( -0.60f, -0.20f, 0.20f, 0.60f ));
			
			lvSideward.AddLabel(fsStrongleft);
			lvSideward.AddLabel(fsMediumleft);
			lvSideward.AddLabel(fsMediumright);
			lvSideward.AddLabel(fsStrongright);
			lvSideward.AddLabel(fsZero0);
			
			
			// Definition for Variable: up
			LinguisticVariable lvUp = new LinguisticVariable("up", -1.00f, 1.00f );
			FuzzySet fsStrongdown = new FuzzySet("strongDown", new TrapezoidalFunction( -0.80f, -0.40f,  TrapezoidalFunction.EdgeType.Right ));
			FuzzySet fsMediumdown = new FuzzySet("mediumDown", new TrapezoidalFunction( -0.90f, -0.60f, -0.40f, 0.00f ));
			FuzzySet fsMediumup = new FuzzySet("mediumUp", new TrapezoidalFunction( 0.00f, 0.40f, 0.60f, 0.90f ));
			FuzzySet fsStrongup = new FuzzySet("strongUp", new TrapezoidalFunction( 0.40f, 0.80f,  TrapezoidalFunction.EdgeType.Left ));
			FuzzySet fsZero1 = new FuzzySet("zero", new TrapezoidalFunction( -0.60f, -0.20f, 0.20f, 0.60f ));
			
			lvUp.AddLabel(fsStrongdown);
			lvUp.AddLabel(fsMediumdown);
			lvUp.AddLabel(fsMediumup);
			lvUp.AddLabel(fsStrongup);
			lvUp.AddLabel(fsZero1);
			
			
			// Definition for Variable: rotation
			LinguisticVariable lvRotation = new LinguisticVariable("rotation", -1.00f, 1.00f );
			FuzzySet fsStrongleft0 = new FuzzySet("strongLeft", new TrapezoidalFunction( -0.80f, -0.40f,  TrapezoidalFunction.EdgeType.Right ));
			FuzzySet fsMediumleft0 = new FuzzySet("mediumLeft", new TrapezoidalFunction( -0.90f, -0.60f, -0.40f, 0.00f ));
			FuzzySet fsMediumright0 = new FuzzySet("mediumRight", new TrapezoidalFunction( 0.00f, 0.40f, 0.60f, 0.90f ));
			FuzzySet fsStrongright0 = new FuzzySet("strongRight", new TrapezoidalFunction( 0.40f, 0.80f,  TrapezoidalFunction.EdgeType.Left ));
			FuzzySet fsZero2 = new FuzzySet("zero", new TrapezoidalFunction( -0.60f, -0.20f, 0.20f, 0.60f ));
			
			lvRotation.AddLabel(fsStrongleft0);
			lvRotation.AddLabel(fsMediumleft0);
			lvRotation.AddLabel(fsMediumright0);
			lvRotation.AddLabel(fsStrongright0);
			lvRotation.AddLabel(fsZero2);
			
			
			// Definition for Variable: backwardSpeed
			LinguisticVariable lvBackwardspeed = new LinguisticVariable("backwardSpeed", -1.00f, 1.00f );
			FuzzySet fsStrongforward0 = new FuzzySet("strongForward", new TrapezoidalFunction( -1.00f, -1.00f, -1.00f, -0.70f ));
			FuzzySet fsMediumforward0 = new FuzzySet("mediumForward", new TrapezoidalFunction( -1.00f, -0.85f, -0.60f, -0.25f ));
			FuzzySet fsMediumbackward0 = new FuzzySet("mediumBackward", new TrapezoidalFunction( 0.25f, 0.60f, 0.85f, 1.00f ));
			FuzzySet fsStrongbackward0 = new FuzzySet("strongBackward", new TrapezoidalFunction( 0.70f, 1.00f, 1.00f, 1.00f ));
			FuzzySet fsZero3 = new FuzzySet("zero", new TrapezoidalFunction( -0.80f, -0.50f, 0.50f, 0.80f ));
			
			lvBackwardspeed.AddLabel(fsStrongforward0);
			lvBackwardspeed.AddLabel(fsMediumforward0);
			lvBackwardspeed.AddLabel(fsMediumbackward0);
			lvBackwardspeed.AddLabel(fsStrongbackward0);
			lvBackwardspeed.AddLabel(fsZero3);
			
			
			// Definition for Variable: sidewardSpeed
			LinguisticVariable lvSidewardspeed = new LinguisticVariable("sidewardSpeed", -1.00f, 1.00f );
			FuzzySet fsStrongleft1 = new FuzzySet("strongLeft", new TrapezoidalFunction( -1.00f, -1.00f, -1.00f, -0.70f ));
			FuzzySet fsMediumleft1 = new FuzzySet("mediumLeft", new TrapezoidalFunction( -1.00f, -0.85f, -0.60f, -0.25f ));
			FuzzySet fsMediumright1 = new FuzzySet("mediumRight", new TrapezoidalFunction( 0.25f, 0.60f, 0.85f, 1.00f ));
			FuzzySet fsStrongright1 = new FuzzySet("strongRight", new TrapezoidalFunction( 0.70f, 1.00f, 1.00f, 1.00f ));
			FuzzySet fsZero4 = new FuzzySet("zero", new TrapezoidalFunction( -0.80f, -0.50f, 0.50f, 0.80f ));
			
			lvSidewardspeed.AddLabel(fsStrongleft1);
			lvSidewardspeed.AddLabel(fsMediumleft1);
			lvSidewardspeed.AddLabel(fsMediumright1);
			lvSidewardspeed.AddLabel(fsStrongright1);
			lvSidewardspeed.AddLabel(fsZero4);
			
			
			// Definition for Variable: upSpeed
			LinguisticVariable lvUpspeed = new LinguisticVariable("upSpeed", -1.00f, 1.00f );
			FuzzySet fsStrongdown0 = new FuzzySet("strongDown", new TrapezoidalFunction( -1.00f, -1.00f, -1.00f, -0.70f ));
			FuzzySet fsMediumdown0 = new FuzzySet("mediumDown", new TrapezoidalFunction( -1.00f, -0.85f, -0.60f, -0.25f ));
			FuzzySet fsMediumup0 = new FuzzySet("mediumUp", new TrapezoidalFunction( 0.25f, 0.60f, 0.85f, 1.00f ));
			FuzzySet fsStrongup0 = new FuzzySet("strongUp", new TrapezoidalFunction( 0.70f, 1.00f, 1.00f, 1.00f ));
			FuzzySet fsZero5 = new FuzzySet("zero", new TrapezoidalFunction( -0.80f, -0.50f, 0.50f, 0.80f ));
			
			lvUpspeed.AddLabel(fsStrongdown0);
			lvUpspeed.AddLabel(fsMediumdown0);
			lvUpspeed.AddLabel(fsMediumup0);
			lvUpspeed.AddLabel(fsStrongup0);
			lvUpspeed.AddLabel(fsZero5);
			
			
			// Definition for Variable: rotationSpeed
			LinguisticVariable lvRotationspeed = new LinguisticVariable("rotationSpeed", -1.00f, 1.00f );
			FuzzySet fsStrongleft2 = new FuzzySet("strongLeft", new TrapezoidalFunction( -1.00f, -1.00f, -1.00f, -0.70f ));
			FuzzySet fsMediumleft2 = new FuzzySet("mediumLeft", new TrapezoidalFunction( -1.00f, -0.85f, -0.60f, -0.25f ));
			FuzzySet fsMediumright2 = new FuzzySet("mediumRight", new TrapezoidalFunction( 0.25f, 0.60f, 0.85f, 1.00f ));
			FuzzySet fsStrongright2 = new FuzzySet("strongRight", new TrapezoidalFunction( 0.70f, 1.00f, 1.00f, 1.00f ));
			FuzzySet fsZero6 = new FuzzySet("zero", new TrapezoidalFunction( -0.80f, -0.50f, 0.50f, 0.80f ));
			
			lvRotationspeed.AddLabel(fsStrongleft2);
			lvRotationspeed.AddLabel(fsMediumleft2);
			lvRotationspeed.AddLabel(fsMediumright2);
			lvRotationspeed.AddLabel(fsStrongright2);
			lvRotationspeed.AddLabel(fsZero6);
			
			
			// Create the database
			Database fuzzyDb = new Database();
			fuzzyDb.AddVariable( lvBackward );
			fuzzyDb.AddVariable( lvSideward );
			fuzzyDb.AddVariable( lvUp );
			fuzzyDb.AddVariable( lvRotation );
			fuzzyDb.AddVariable( lvBackwardspeed );
			fuzzyDb.AddVariable( lvSidewardspeed );
			fuzzyDb.AddVariable( lvUpspeed );
			fuzzyDb.AddVariable( lvRotationspeed );
			
			// Create the inference system
			infSystem = new InferenceSystem( fuzzyDb, new CentroidDefuzzifier( 1000 ));
			
			// Defintion of Rules
			infSystem.NewRule("Rule 1", "IF backward IS strongForward THEN backwardSpeed IS strongForward");
			infSystem.NewRule("Rule 2", "IF backward IS mediumForward THEN backwardSpeed IS mediumForward");
			infSystem.NewRule("Rule 3", "IF backward IS mediumBackward THEN backwardSpeed IS strongBackward");
			infSystem.NewRule("Rule 4", "IF backward IS strongBackward THEN backwardSpeed IS strongBackward");
			infSystem.NewRule("Rule 5", "IF backward IS zero THEN backwardSpeed IS zero");
			infSystem.NewRule("Rule 6", "IF sideward IS strongLeft THEN sidewardSpeed IS strongLeft");
			infSystem.NewRule("Rule 7", "IF sideward IS mediumLeft THEN sidewardSpeed IS mediumLeft");
			infSystem.NewRule("Rule 8", "IF sideward IS mediumRight THEN sidewardSpeed IS mediumRight");
			infSystem.NewRule("Rule 9", "IF sideward IS strongRight THEN sidewardSpeed IS strongRight");
			infSystem.NewRule("Rule 10", "IF sideward IS zero THEN sidewardSpeed IS zero");
			infSystem.NewRule("Rule 11", "IF backward IS mediumBackward THEN backwardSpeed IS strongBackward");
			infSystem.NewRule("Rule 12", "IF up IS strongDown THEN upSpeed IS strongDown");
			infSystem.NewRule("Rule 13", "IF up IS mediumDown THEN upSpeed IS mediumDown");
			infSystem.NewRule("Rule 14", "IF up IS mediumUp THEN upSpeed IS mediumUp");
			infSystem.NewRule("Rule 15", "IF up IS strongUp THEN upSpeed IS strongUp");
			infSystem.NewRule("Rule 16", "IF up IS zero THEN upSpeed IS zero");
			infSystem.NewRule("Rule 17", "IF rotation IS strongLeft THEN rotationSpeed IS strongLeft");
			infSystem.NewRule("Rule 18", "IF rotation IS mediumLeft THEN rotationSpeed IS mediumLeft");
			infSystem.NewRule("Rule 19", "IF rotation IS mediumRight THEN rotationSpeed IS mediumRight");
			infSystem.NewRule("Rule 20", "IF rotation IS strongRight THEN rotationSpeed IS strongRight");
			infSystem.NewRule("Rule 21", "IF rotation IS zero THEN rotationSpeed IS zero");
			infSystem.NewRule("Rule 22", "IF backward IS strongForward AND sideward IS strongLeft THEN backwardSpeed IS strongForward");
			infSystem.NewRule("Rule 23", "IF backward IS strongForward AND sideward IS strongLeft THEN sidewardSpeed IS strongLeft");
			infSystem.NewRule("Rule 24", "IF backward IS strongForward AND sideward IS strongLeft THEN rotationSpeed IS strongLeft");
			infSystem.NewRule("Rule 25", "IF backward IS mediumForward AND sideward IS mediumLeft THEN backwardSpeed IS mediumForward");
			infSystem.NewRule("Rule 26", "IF backward IS mediumForward AND sideward IS mediumLeft THEN sidewardSpeed IS mediumLeft");
			infSystem.NewRule("Rule 27", "IF backward IS mediumForward AND sideward IS mediumLeft THEN rotationSpeed IS mediumLeft");
			infSystem.NewRule("Rule 28", "IF backward IS strongForward AND sideward IS strongRight THEN backwardSpeed IS strongForward");
			infSystem.NewRule("Rule 29", "IF backward IS strongForward AND sideward IS strongRight THEN sidewardSpeed IS strongRight");
			infSystem.NewRule("Rule 30", "IF backward IS strongForward AND sideward IS strongRight THEN rotationSpeed IS strongRight");
			infSystem.NewRule("Rule 31", "IF backward IS mediumForward AND sideward IS mediumRight THEN backwardSpeed IS mediumForward");
			infSystem.NewRule("Rule 32", "IF backward IS mediumForward AND sideward IS mediumRight THEN sidewardSpeed IS mediumRight");
			infSystem.NewRule("Rule 33", "IF backward IS mediumForward AND sideward IS mediumRight THEN rotationSpeed IS mediumRight");
			infSystem.NewRule("Rule 34", "IF backward IS mediumForward AND sideward IS strongRight THEN backwardSpeed IS mediumForward");
			infSystem.NewRule("Rule 35", "IF backward IS mediumForward AND sideward IS strongRight THEN sidewardSpeed IS strongRight");
			infSystem.NewRule("Rule 36", "IF backward IS mediumForward AND sideward IS strongRight THEN rotationSpeed IS strongRight");
			infSystem.NewRule("Rule 37", "IF backward IS mediumForward AND sideward IS strongLeft THEN backwardSpeed IS mediumForward");
			infSystem.NewRule("Rule 38", "IF backward IS mediumForward AND sideward IS strongLeft THEN sidewardSpeed IS strongLeft");
			infSystem.NewRule("Rule 39", "IF backward IS mediumForward AND sideward IS strongLeft THEN rotationSpeed IS strongLeft");
			infSystem.NewRule("Rule 40", "IF backward IS mediumForward AND sideward IS zero THEN backwardSpeed IS mediumForward");
			infSystem.NewRule("Rule 41", "IF backward IS mediumForward AND sideward IS zero THEN sidewardSpeed IS zero");
			infSystem.NewRule("Rule 42", "IF backward IS mediumForward AND sideward IS zero THEN rotationSpeed IS zero");
			infSystem.NewRule("Rule 43", "IF backward IS strongForward AND sideward IS zero THEN backwardSpeed IS strongForward");
			infSystem.NewRule("Rule 44", "IF backward IS strongForward AND sideward IS zero THEN sidewardSpeed IS zero");
			infSystem.NewRule("Rule 45", "IF backward IS strongForward AND sideward IS zero THEN rotationSpeed IS zero");
			infSystem.NewRule("Rule 46", "IF backward IS strongForward AND sideward IS mediumLeft THEN backwardSpeed IS strongForward");
			infSystem.NewRule("Rule 47", "IF backward IS strongForward AND sideward IS mediumLeft THEN sidewardSpeed IS mediumLeft");
			infSystem.NewRule("Rule 48", "IF backward IS strongForward AND sideward IS mediumLeft THEN rotationSpeed IS strongLeft");
			infSystem.NewRule("Rule 49", "IF backward IS strongForward AND sideward IS mediumRight THEN backwardSpeed IS strongForward");
			infSystem.NewRule("Rule 50", "IF backward IS strongForward AND sideward IS mediumRight THEN sidewardSpeed IS mediumRight");
			infSystem.NewRule("Rule 51", "IF backward IS strongForward AND sideward IS mediumRight THEN rotationSpeed IS strongRight");
			infSystem.NewRule("Rule 52", "IF backward IS mediumBackward THEN backwardSpeed IS mediumBackward");
			
		}
		
		public FISResult DoInference(float backward, float sideward, float up, float rotation) 
		{

			// Set inputs
			infSystem.SetInput("backward", backward);
			infSystem.SetInput("sideward", sideward);
			infSystem.SetInput("up", up);
			infSystem.SetInput("rotation", rotation);
			
			
			// Set outputs
			float backwardSpeed = infSystem.Evaluate("backwardSpeed");
			float sidewardSpeed = infSystem.Evaluate("sidewardSpeed");
			float upSpeed = infSystem.Evaluate("upSpeed");
			float rotationSpeed = infSystem.Evaluate("rotationSpeed");
			
			
			return new FISResult(backwardSpeed, sidewardSpeed, upSpeed, rotationSpeed);
		}
	}
}