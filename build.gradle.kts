plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
}

ihmc {
   group = "us.ihmc"
   version = "0.7.4"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("com.google.guava:guava:18.0")
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")

   api("us.ihmc:euclid:0.18.1")
   api("us.ihmc:euclid-geometry:0.18.1")
   api("us.ihmc:euclid-frame:0.18.1")
   api("us.ihmc:mecano:0.11.1")
   api("us.ihmc:ihmc-commons:0.31.0")
   api("us.ihmc:ihmc-native-library-loader:1.3.1")
   api("us.ihmc:ihmc-yovariables:0.9.15")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))

   api("org.ejml:ejml-simple:0.39")
   api("net.sf.trove4j:trove4j:3.0.3")
   api("org.apache.commons:commons-math3:3.6.1")

   api("us.ihmc:ihmc-commons-testing:0.31.0")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:simulation-construction-set:0.22.3")
}
