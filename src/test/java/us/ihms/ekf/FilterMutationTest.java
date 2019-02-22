package us.ihms.ekf;

import java.io.IOException;
import java.util.function.Predicate;

import com.google.common.collect.ImmutableSet;
import com.google.common.reflect.ClassPath;
import com.google.common.reflect.ClassPath.ClassInfo;

import us.ihmc.commons.MutationTestFacilitator;

public class FilterMutationTest
{
   private static final Predicate<? super ClassInfo> testClassFilter = ci -> ci.getName().endsWith("Test") && ci.getName().startsWith("us.ihms.ekf")
         && !ci.getName().equals(FilterMutationTest.class.getName());

   public static void main(String[] args) throws IOException
   {
      MutationTestFacilitator mutationTestFacilitator = new MutationTestFacilitator();
      mutationTestFacilitator.addPackagePathsToMutate("us.ihmc.ekf.*");

      ImmutableSet<ClassInfo> topLevelClasses = ClassPath.from(FilterMutationTest.class.getClassLoader()).getAllClasses();
      topLevelClasses.stream().filter(testClassFilter).forEach(ci -> {
         mutationTestFacilitator.addClassesToMutate(ci.load());
         System.out.println("Adding test " + ci.getName());
      });

      mutationTestFacilitator.doMutationTest();
   }
}
