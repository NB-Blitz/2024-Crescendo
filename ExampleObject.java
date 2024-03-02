// This class is an example object for teaching java syntax

/*--------------------------------------------------------------------------
 * Imports
 --------------------------------------------------------------------------*/
 // 

/*--------------------------------------------------------------------------
 * Class/Object Definition
 --------------------------------------------------------------------------*/
// public - This is the access modifier for the class. Public allows this class to be defined in other classes
// class - This keyword is required for creating a new class
// ExampleObject - This is the name of the object. Object name should have capital letters for the first letter of each word.
//                 Object names cannot have spaces in them.
// {} - These brackets contain all of the code for this class. Anything outside of these brackets will not be part of
//      this class.
public class ExampleObject {
    // This is the section of the object where you put instance variables for the object.
    // Instance variables are variables that the object needs to keep track of its state, configurations, or features
    // that make it unique.
    // Example: A vehicle object could have an instance variable numberOfWheels of type integer.

    /*--------------------------------------------------------------------------
    * Instance Variables
    --------------------------------------------------------------------------*/
    // private - This is the access modifier for this instance variable. private means that this
    //           variable cannot be accessed outside of this class which is what you would typically want.
    // int - This is the variable type of the instance variable. This limits what the variable can be set to.
    //       The variable type can be either primative datatypes (int, double, etc) or Objects (ExampleObject, String, etc.)
    //       Notice that primative datatypes are not capitalized while objects are.
    // instanceVariable - This is the name of the variable. What you call this has no impact on functinality.
    //                    However, you want to give it a name that makes it easy to tell what it is being used for.
    //                    Notice that the first word is not capitalized, but the following words are. This is called
    //                    cammel case.
    private int instanceVariable1;

    private final String INSTANCE_VARIABLE_2 = "Example String\n";

    /*--------------------------------------------------------------------------
    * Constructor
    --------------------------------------------------------------------------*/
    // The Constructor is for initializing instance variables to make the object instance
    // unique for this specific use case. The Constructor is a special example of a function.
    // public - This is the access modifier for the Constructor. public allows other classes to
    //          create an instance of ExampleObject.
    // Note: Contstructors do not have a return type (e.g. int, void, double, etc.)
    // ExampleObject - This is the name of the Constructor function. The function name for
    //                 the Constructor must be the same as the name of the object it is 
    //                 associated with.
    // int - This is the datatype of the parameter being passed into the Constructor. This type typically
    //       matches the datatype of the instance variable above that you are initializing.
    // parameter1 - This is the variable name of the parameter being passed into the function. This is what
    //              you will type to access the value that was passed in as a parameter. 
    public ExampleObject(int parameter1){
        // This line sets the instance variable instancVariable1 to the value of 
        // the Constructor function parameter parameter1.
        instanceVariable1 = parameter1;
    }

    public void printExampleObject()
    {
        System.out.print("INSTANCE_VARIABLE_2=" + INSTANCE_VARIABLE_2 + "\n" +
                         "instanceVariable1=" + instanceVariable1 + "\n" +
                         "exampleFunction(2) output=" + exampleFunction(2));
    }

    private int exampleFunction(int addition)
    {
        return instanceVariable1 + addition;
    }


}
