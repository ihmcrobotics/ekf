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
   api("org.ejml:dense64:0.30")
   api("org.ejml:core:0.30")

   api("us.ihmc:euclid:0.14.1")
   api("us.ihmc:euclid-geometry:0.14.1")
   api("us.ihmc:euclid-frame:0.14.1")
   api("us.ihmc:mecano:0.4.0")
   api("us.ihmc:ihmc-commons:0.29.0")
   api("us.ihmc:ihmc-native-library-loader:1.2.1")
   api("us.ihmc:ihmc-yovariables:0.6.0")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:ihmc-commons-testing:0.29.0")
   api("org.ejml:simple:0.30")
   api("net.sf.trove4j:trove4j:3.0.3")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:simulation-construction-set:0.16.0")
}
