using AForge.Fuzzy;

namespace Fuzzy
{	
	// Represents an output of the FIS
    public class FISResult
    {	
		// output parameters
		public float backwardSpeed { get; private set; }
		public float sidewardSpeed { get; private set; }
		public float upSpeed { get; private set; }
		public float rotationSpeed { get; private set; }
		
		
		// Constructor
	    internal FISResult(float backwardSpeed, float sidewardSpeed, float upSpeed, float rotationSpeed) {
			// variable initialisation
			this.backwardSpeed = backwardSpeed;
			this.sidewardSpeed = sidewardSpeed;
			this.upSpeed = upSpeed;
			this.rotationSpeed = rotationSpeed;
			
		}

		public override string ToString() {
            return "backwardSpeed: " + backwardSpeed + "; " + "sidewardSpeed: " + sidewardSpeed + "; " + "upSpeed: " + upSpeed + "; " + "rotationSpeed: " + rotationSpeed + "; ";
        }

	}
}