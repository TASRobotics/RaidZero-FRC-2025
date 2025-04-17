# Style guide

## Notes

- Before pushing the code, make sure to format the document!
    - `Ctrl + Shift + P` or `⌘ + Shift + P` → `Format Document`

## Rules

- Constants
    - All caps with underscores (eg: `SOME_CONSTANT_VALUE`)
- Member variables
    - No "m" with camelCase (eg: `someMemberVariable` instead of `mSomeMemberVariable`)
- Subsystems should be in the form of a singleton class
- Class method order: Getters, setters, other methods (alphabetically)
- Member definition order
    - Grouped logically (in an order that makes sense), but Talon configs then subsystem static object getter comes last
- Sample order:
    1. Some Motor
    2. Some sensor
    3. Voltage/DutyCycle configs (If applicable)
    4. Static subsystem instance
    5. Private class constructor
    6. Getters
    7. Setters
    8. Other methods (Orderd based on functionality)
    9. Subsystem instance getter
    10. Subsystem configs (Eg, Swerve will have AutoBuilder config)
    11. Device configs (If applicable)
- Subsystem constructors are private and will be called within the class
- Import statements: let Java RedHat extension take care of it
- Spaces in code where necessary (Eg, between methods, between operations such as + or -)
- Braces are on the same line as method declarations
- The `this` keyword should be used for ***ALL*** member objects/methods
- ***Always*** add Javadoc to methods!!

## Code styling

- Opening braces should be on the same line as method declaration

Example:

```java
public void someMethod(boolean param1, boolean param2) {
    // Method body
}
```

- Method chaining should split over multiple lines

Example:

```java
public void initialize() {
    someObject.withInteger(1)
              .withBoolean(true)
              .withDouble(0.1);
}
```

- Parameters should be split over multiple lines if the line gets too long
    - Method call should still stay on the same line

Example:

```java
public void initialize() {
    someObject.setConstants(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
    );
}
```

- Indentation
    - Tabs, 4 spaces


## Setters & Getters

- The method name should be the same as the object being modified
    - Parameter name should also be the same as the name of the object being modified

Eg:

```java
private boolean someBoolean

public void setSomeBoolean(boolean someBoolean) {
    this.someBoolean = someBoolean
}

public void getSomeBoolean() {
    return this.someBoolean;
}

public void someMethod() {
    int a = 0;

    a += (this.someBoolean ? 2 : 4);
}
```

## Importing constants

- Statically import the class of the subsystem constants in code instead of doing `Constants.Subsystem.Subclass.VALUE`

For example, we have a value called `MAX_VEL` inside the `Subpart` class.

```java
public class Constants {
    public static class Subsystem {
        public static class Subpart {
            public static final double MAX_VEL = 2.5;
        }
    }
}
```

Instead of doing this:

```java
import raidzero.robot.Constants;

// Inside a method
motor.set(Constants.Subsystem.Subpart.MAX_VEL);
```

or,

```java
import raidzero.robot.Constants.Subsystem.Subpart;

// Inside a method
motor.set(Subpart.MAX_VEL);
```

do this:

```java
import static raidzero.robot.Constants.Subsystem.Subpart.*;

// Inside a method
motor.set(MAX_VEL);
```