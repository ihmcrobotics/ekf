package us.ihmc.ekf;

import java.io.IOException;
import java.util.function.Predicate;

import com.google.common.reflect.ClassPath;
import com.google.common.reflect.ClassPath.ClassInfo;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.log.LogTools;

public class FilterMutationTest
{
   private static final Predicate<? super ClassInfo> testClassFilter = ci -> ci.getName().endsWith("Test");
   private static final Predicate<? super ClassInfo> mutationClassFilter = ci -> !testClassFilter.test(ci);

   public static void main(String[] args) throws IOException
   {
      MutationTestFacilitator mutationTestFacilitator = new MutationTestFacilitator();
      ClassPath classPath = ClassPath.from(FilterMutationTest.class.getClassLoader());

      LogTools.info("Adding classes to mutate:");
      classPath.getTopLevelClassesRecursive("us.ihmc.ekf.filter").stream().filter(mutationClassFilter).forEach(ci -> {
         mutationTestFacilitator.addClassesToMutate(ci.load());
         System.out.println(ci.getName());
      });

      LogTools.info("Adding test classes:");
      classPath.getTopLevelClassesRecursive("us.ihmc.ekf.filter").stream().filter(testClassFilter).forEach(ci -> {
         mutationTestFacilitator.addTestClassesToRun(ci.load());
         System.out.println(ci.getName());
      });

      mutationTestFacilitator.doMutationTest();
   }
}
