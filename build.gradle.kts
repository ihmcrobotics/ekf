plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   group = "us.ihmc"
   version = "0.7.9"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("com.google.guava:guava:18.0")
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-ddense:0.39")

   api("us.ihmc:euclid:0.22.2")
   api("us.ihmc:euclid-geometry:0.22.2")
   api("us.ihmc:euclid-frame:0.22.2")
   api("us.ihmc:mecano:17-0.19.0")
   api("us.ihmc:ihmc-commons:0.34.0")
   api("us.ihmc:ihmc-native-library-loader:2.0.3")
   api("us.ihmc:ihmc-yovariables:0.13.4")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))

   api("org.ejml:ejml-simple:0.39")
   api("net.sf.trove4j:trove4j:3.0.3")
   api("org.apache.commons:commons-math3:3.6.1")

   api("us.ihmc:ihmc-commons-testing:0.34.0")
}

visualizersDependencies {
   api(ihmc.sourceSetProject("main"))

   api("us.ihmc:simulation-construction-set:0.25.3")
}
