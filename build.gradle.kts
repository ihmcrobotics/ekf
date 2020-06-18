plugins {
   id("us.ihmc.ihmc-build") version "0.20.1"
   id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.8"
}

ihmc {
   group = "us.ihmc"
   version = "0.5.0"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("com.google.guava:guava:18.0")
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")

   api("us.ihmc:euclid:0.14.2-ejml-0.39-beta-1")
   api("us.ihmc:euclid-geometry:0.14.2-ejml-0.39-beta-1")
   api("us.ihmc:euclid-frame:0.14.2-ejml-0.39-beta-1")
   api("us.ihmc:mecano:0.5.0-ejml-0.39-beta-1")
   api("us.ihmc:ihmc-commons:0.30.0")
   api("us.ihmc:ihmc-native-library-loader:1.2.1")
   api("us.ihmc:ihmc-yovariables:0.7.0-ejml-0.39-beta-1")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))

   api("org.ejml:ejml-simple:0.39")
   api("net.sf.trove4j:trove4j:3.0.3")

   api("us.ihmc:ihmc-commons-testing:0.30.0")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:simulation-construction-set:0.18.0-ejml-0.39-beta-1")
}
