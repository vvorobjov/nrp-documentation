To generate the documentation of an XML Schema, apply the XSLT Transformation xsd2rst.xsl in this folder to it.

For this, you have to use an XSLT-Processor. You can either download one or there are free online services that perform the transformation.
The transformation was tested using msxsl on Windows. Using this tool, you can generate the rst spec as follows:

msxsl <path to schema.xsd> xsd2rst.xsl title="<A title for the generated rst document>" -o <path to generated .rst>