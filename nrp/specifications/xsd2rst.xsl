<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<xsl:stylesheet 
 xmlns:xsl="http://www.w3.org/1999/XSL/Transform" 
 xmlns="http://www.w3.org/1999/xhtml" 
 xmlns:html="http://www.w3.org/1999/xhtml" 
 xmlns:xsd="http://www.w3.org/2001/XMLSchema" 
 xmlns:ppp="http://titanium.dstc.edu.au/xml/xs3p" 
 version="1.0" 
 exclude-result-prefixes="xsd ppp html">
   <xsl:output 
    method="text" 
    encoding="ISO-8859-1"
    standalone="yes"
    version="1.0"
    doctype-public="-//W3C//DTD XHTML 1.0 Strict//EN" 
    doctype-system="http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd"/>
  <xsl:key name="type" match="/xsd:schema/xsd:complexType | /xsd:schema/xsd:simpleType | /xsd:schema/xsd:redefine/xsd:complexType | /xsd:schema/xsd:redefine/xsd:simpleType" use="@name" />
  <xsl:key name="complexType" match="/xsd:schema/xsd:complexType | /xsd:schema/xsd:redefine/xsd:complexType" use="@name" />
  <xsl:key name="simpleType" match="/xsd:schema/xsd:simpleType | /xsd:schema/xsd:redefine/xsd:simpleType" use="@name" />
  <xsl:key name="attributeGroup" match="/xsd:schema/xsd:attributeGroup | /xsd:schema/xsd:redefine/xsd:attributeGroup" use="@name" />
  <xsl:key name="group" match="/xsd:schema/xsd:group | /xsd:schema/xsd:redefine/xsd:group" use="@name" />
  <xsl:key name="attribute" match="/xsd:schema/xsd:attribute" use="@name" />
  <xsl:key name="element" match="/xsd:schema/xsd:element" use="@name" />
  <xsl:param name="title"></xsl:param>
  <xsl:variable name="XSD_NS">http://www.w3.org/2001/XMLSchema</xsl:variable>
  <xsl:variable name="XML_NS">http://www.w3.org/XML/1998/namespace</xsl:variable>
  <xsl:variable name="ORIGINAL_SCHEMA" select="/xsd:schema"/>
  <xsl:template name="PrintHeaderLine">
    <xsl:param name="header"/>
    <xsl:param name="length"/>
    <xsl:if test="$length &gt; 0">
      <xsl:value-of select="$header"/>
      <xsl:call-template name="PrintHeaderLine">
        <xsl:with-param name="header" select="$header"/>
        <xsl:with-param name="length" select="$length - 1"/>
      </xsl:call-template>
    </xsl:if>
  </xsl:template>
  <xsl:template match="/xsd:schema">
    <xsl:value-of select="$title"/>
    <xsl:text>
</xsl:text>
    <xsl:call-template name="PrintHeaderLine">
    <xsl:with-param name="header" select="'='"/>
    <xsl:with-param name="length" select="string-length($title)"/>
  </xsl:call-template>

Schema Document Properties
--------------------------
<xsl:apply-templates select="." mode="properties"/>

Declared Namespaces
-------------------
<xsl:apply-templates select="." mode="namespaces"/>
<xsl:if test="xsd:redefine">

Redefined Schema Components
---------------------------
<xsl:apply-templates select="xsd:redefine/xsd:simpleType | xsd:redefine/xsd:complexType | xsd:redefine/xsd:attributeGroup | xsd:redefine/xsd:group" mode="topSection"/>
</xsl:if>    
<xsl:if test="xsd:attribute or xsd:element">

Global Declarations
-------------------
   <xsl:apply-templates select="xsd:attribute | xsd:element" mode="topSection">
     <xsl:sort select="local-name(.)" order="ascending"/>
     <xsl:sort select="@name" order="ascending"/>
   </xsl:apply-templates>
</xsl:if>

<xsl:if test="xsd:attributeGroup or xsd:complexType or xsd:group or xsd:notation or xsd:simpleType">

Global Definitions
------------------
   <xsl:apply-templates select="xsd:attributeGroup | xsd:complexType | xsd:group | xsd:notation | xsd:simpleType" mode="topSection">
     <xsl:sort select="local-name(.)" order="ascending"/>
     <xsl:sort select="@name" order="ascending"/>
   </xsl:apply-templates>
</xsl:if>
</xsl:template>

<xsl:template match="xsd:schema" mode="namespaces">
  <xsl:text>
+-------------------+---------------------------------------------------------------------------------------------------------+
| Prefix            | Namespace                                                                                               |
+===================+=========================================================================================================+
</xsl:text>
  <xsl:if test="namespace::*[local-name(.)='']">
    <xsl:text>| (default)         | </xsl:text>
    <xsl:value-of select="namespace::*[local-name(.)='']"/>
    <xsl:call-template name="PrintHeaderLine">
      <xsl:with-param name="header" select="' '"/>
      <xsl:with-param name="length" select="104 - string-length(namespace::*[local-name(.)=''])"/>
    </xsl:call-template>
    <xsl:text>|
</xsl:text>
  </xsl:if>
  <xsl:for-each select="namespace::*[local-name(.)!='']">
    <xsl:variable name="prefix" select="local-name(.)"/>
    <xsl:variable name="ns" select="."/>
    <xsl:text>+-------------------+---------------------------------------------------------------------------------------------------------+
</xsl:text>
    <xsl:text>| </xsl:text>
    <xsl:value-of select="$prefix"/>
    <xsl:call-template name="PrintHeaderLine">
      <xsl:with-param name="header" select="' '"/>
      <xsl:with-param name="length" select="18 - string-length($prefix)"/>
    </xsl:call-template>
    <xsl:text>| </xsl:text>
    <xsl:value-of select="$ns"/>
    <xsl:call-template name="PrintHeaderLine">
      <xsl:with-param name="header" select="' '"/>
      <xsl:with-param name="length" select="104 - string-length($ns)"/>
    </xsl:call-template>
    <xsl:text>|
</xsl:text>
  </xsl:for-each>
  <xsl:text>+-------------------+---------------------------------------------------------------------------------------------------------+

</xsl:text>
</xsl:template>

   <xsl:template match="xsd:*[@name]" mode="topSection">
      <!-- Header -->
      <xsl:call-template name="ComponentSectionHeader">
         <xsl:with-param name="component" select="."/>
      </xsl:call-template>

      <!-- Hierarchy table (for types and elements) -->
      <xsl:apply-templates select="." mode="hierarchy"/>
     <xsl:text>
</xsl:text>

      <!-- Properties table -->
      <xsl:apply-templates select="." mode="properties"/>
     <xsl:text>
</xsl:text>

      <!-- XML Instance Representation table -->
      <xsl:call-template name="SampleInstanceTable">
         <xsl:with-param name="component" select="."/>
      </xsl:call-template>
   </xsl:template>
  <xsl:template name="ComponentSectionHeader">
    <xsl:param name="component"/>
    <xsl:variable name="componentID">
      <xsl:call-template name="GetComponentID">
        <xsl:with-param name="component" select="$component"/>
      </xsl:call-template>
    </xsl:variable>
    <xsl:variable name="componentDescription">
      <xsl:call-template name="GetComponentDescription">
        <xsl:with-param name="component" select="$component"/>
      </xsl:call-template>
    </xsl:variable>
    <xsl:variable name="componentTermRef">
      <xsl:call-template name="GetComponentTermRef">
        <xsl:with-param name="component" select="$component"/>
      </xsl:call-template>
    </xsl:variable>
    <xsl:text>
</xsl:text>
    <!-- Description -->
    <xsl:choose>
      <xsl:when test="$componentTermRef != ''">
        <xsl:call-template name="PrintGlossaryTermRef">
          <xsl:with-param name="code" select="$componentTermRef"/>
          <xsl:with-param name="term" select="$componentDescription"/>
        </xsl:call-template>
      </xsl:when>
      <xsl:otherwise>
        <xsl:value-of select="$componentDescription"/>
      </xsl:otherwise>
    </xsl:choose>
    <xsl:text>: </xsl:text>
    <xsl:value-of select="$component/@name"/>
    <xsl:text>
</xsl:text>
    <xsl:call-template name="PrintHeaderLine">
      <xsl:with-param name="header" select="'^'"/>
      <xsl:with-param name="length" select="string-length($componentDescription) + string-length($component/@name) + 2"/>
    </xsl:call-template>
    <xsl:text>
</xsl:text>
  </xsl:template>

  <xsl:template name="Glossary">
      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">Abstract</xsl:with-param>
         <xsl:with-param name="term">Abstract</xsl:with-param>
         <xsl:with-param name="description">
            <xsl:text>(Applies to complex type definitions and element declarations).</xsl:text>
            <xsl:text> An abstract element or complex type cannot used to validate an element instance.</xsl:text>
            <xsl:text> If there is a reference to an abstract element, only element declarations that can substitute the abstract element can be used to validate the instance.</xsl:text>
            <xsl:text> For references to abstract type definitions, only derived types can be used.</xsl:text>
         </xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">All</xsl:with-param>
         <xsl:with-param name="term">All Model Group</xsl:with-param>
         <xsl:with-param name="description">
            <xsl:text>Child elements can be provided </xsl:text>
            <em>
               <xsl:text>in any order</xsl:text>
            </em>
            <xsl:text> in instances.</xsl:text>
         </xsl:with-param>
         <xsl:with-param name="link">http://www.w3.org/TR/xmlschema-1/#element-all</xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">Choice</xsl:with-param>
         <xsl:with-param name="term">Choice Model Group</xsl:with-param>
         <xsl:with-param name="description">
            <em>
               <xsl:text>Only one</xsl:text>
            </em>
            <xsl:text> from the list of child elements and model groups can be provided in instances.</xsl:text>
         </xsl:with-param>
         <xsl:with-param name="link">http://www.w3.org/TR/xmlschema-1/#element-choice</xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">CollapseWS</xsl:with-param>
         <xsl:with-param name="term">Collapse Whitespace Policy</xsl:with-param>
         <xsl:with-param name="description">Replace tab, line feed, and carriage return characters with space character (Unicode character 32). Then, collapse contiguous sequences of space characters into single space character, and remove leading and trailing space characters.</xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">ElemBlock</xsl:with-param>
         <xsl:with-param name="term">Disallowed Substitutions</xsl:with-param>
         <xsl:with-param name="description">
            <xsl:text>(Applies to element declarations).</xsl:text>
            <xsl:text> If </xsl:text>
            <em>substitution</em>
            <xsl:text> is specified, then </xsl:text>
            <xsl:call-template name="PrintGlossaryTermRef">
               <xsl:with-param name="code">SubGroup</xsl:with-param>
               <xsl:with-param name="term">substitution group</xsl:with-param>
            </xsl:call-template>
            <xsl:text> members cannot be used in place of the given element declaration to validate element instances.</xsl:text>

            <xsl:text> If </xsl:text>
            <em>derivation methods</em>
            <xsl:text>, e.g. extension, restriction, are specified, then the given element declaration will not validate element instances that have types derived from the element declaration's type using the specified derivation methods.</xsl:text>
            <xsl:text> Normally, element instances can override their declaration's type by specifying an </xsl:text>
            <code>xsi:type</code>
            <xsl:text> attribute.</xsl:text>
         </xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">Key</xsl:with-param>
         <xsl:with-param name="term">Key Constraint</xsl:with-param>
         <xsl:with-param name="description">
            <xsl:text>Like </xsl:text>
            <xsl:call-template name="PrintGlossaryTermRef">
               <xsl:with-param name="code">Unique</xsl:with-param>
               <xsl:with-param name="term">Uniqueness Constraint</xsl:with-param>
            </xsl:call-template>
            <xsl:text>, but additionally requires that the specified value(s) must be provided.</xsl:text>
         </xsl:with-param>
         <xsl:with-param name="link">http://www.w3.org/TR/xmlschema-1/#cIdentity-constraint_Definitions</xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">KeyRef</xsl:with-param>
         <xsl:with-param name="term">Key Reference Constraint</xsl:with-param>
         <xsl:with-param name="description">
            <xsl:text>Ensures that the specified value(s) must match value(s) from a </xsl:text>
            <xsl:call-template name="PrintGlossaryTermRef">
               <xsl:with-param name="code">Key</xsl:with-param>
               <xsl:with-param name="term">Key Constraint</xsl:with-param>
            </xsl:call-template>
            <xsl:text> or </xsl:text>
            <xsl:call-template name="PrintGlossaryTermRef">
               <xsl:with-param name="code">Unique</xsl:with-param>
               <xsl:with-param name="term">Uniqueness Constraint</xsl:with-param>
            </xsl:call-template>
            <xsl:text>.</xsl:text>
         </xsl:with-param>
         <xsl:with-param name="link">http://www.w3.org/TR/xmlschema-1/#cIdentity-constraint_Definitions</xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">ModelGroup</xsl:with-param>
         <xsl:with-param name="term">Model Group</xsl:with-param>
         <xsl:with-param name="description">
            <xsl:text>Groups together element content, specifying the order in which the element content can occur and the number of times the group of element content may be repeated.</xsl:text>
         </xsl:with-param>
         <xsl:with-param name="link">http://www.w3.org/TR/xmlschema-1/#Model_Groups</xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">Nillable</xsl:with-param>
         <xsl:with-param name="term">Nillable</xsl:with-param>
         <xsl:with-param name="description">
            <xsl:text>(Applies to element declarations). </xsl:text>
            <xsl:text>If an element declaration is nillable, instances can use the </xsl:text>
            <code>xsi:nil</code>
            <xsl:text> attribute.</xsl:text>
            <xsl:text> The </xsl:text>
            <code>xsi:nil</code>
            <xsl:text> attribute is the boolean attribute, </xsl:text>
            <em>nil</em>
            <xsl:text>, from the </xsl:text>
            <em>http://www.w3.org/2001/XMLSchema-instance</em>
            <xsl:text> namespace.</xsl:text>
            <xsl:text> If an element instance has an </xsl:text>
            <code>xsi:nil</code>
            <xsl:text> attribute set to true, it can be left empty, even though its element declaration may have required content.</xsl:text>
         </xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">Notation</xsl:with-param>
         <xsl:with-param name="term">Notation</xsl:with-param>
         <xsl:with-param name="description">A notation is used to identify the format of a piece of data. Values of elements and attributes that are of type, NOTATION, must come from the names of declared notations.</xsl:with-param>
         <xsl:with-param name="link">http://www.w3.org/TR/xmlschema-1/#cNotation_Declarations</xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">PreserveWS</xsl:with-param>
         <xsl:with-param name="term">Preserve Whitespace Policy</xsl:with-param>
         <xsl:with-param name="description">Preserve whitespaces exactly as they appear in instances.</xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">TypeFinal</xsl:with-param>
         <xsl:with-param name="term">Prohibited Derivations</xsl:with-param>
         <xsl:with-param name="description">
            <xsl:text>(Applies to type definitions). </xsl:text>
            <xsl:text>Derivation methods that cannot be used to create sub-types from a given type definition.</xsl:text>
         </xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">TypeBlock</xsl:with-param>
         <xsl:with-param name="term">Prohibited Substitutions</xsl:with-param>
         <xsl:with-param name="description">
            <xsl:text>(Applies to complex type definitions). </xsl:text>
            <xsl:text>Prevents sub-types that have been derived using the specified derivation methods from validating element instances in place of the given type definition.</xsl:text>
         </xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">ReplaceWS</xsl:with-param>
         <xsl:with-param name="term">Replace Whitespace Policy</xsl:with-param>
         <xsl:with-param name="description">Replace tab, line feed, and carriage return characters with space character (Unicode character 32).</xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">Sequence</xsl:with-param>
         <xsl:with-param name="term">Sequence Model Group</xsl:with-param>
         <xsl:with-param name="description">
            <xsl:text>Child elements and model groups must be provided </xsl:text>
            <em>
               <xsl:text>in the specified order</xsl:text>
            </em>
            <xsl:text> in instances.</xsl:text>
         </xsl:with-param>
         <xsl:with-param name="link">http://www.w3.org/TR/xmlschema-1/#element-sequence</xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">SubGroup</xsl:with-param>
         <xsl:with-param name="term">Substitution Group</xsl:with-param>
         <xsl:with-param name="description">
            <xsl:text>Elements that are </xsl:text>
            <em>
               <xsl:text>members</xsl:text>
            </em>
            <xsl:text> of a substitution group can be used wherever the </xsl:text>
            <em>
               <xsl:text>head</xsl:text>
            </em>
            <xsl:text> element of the substitution group is referenced.</xsl:text>
         </xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">ElemFinal</xsl:with-param>
         <xsl:with-param name="term">Substitution Group Exclusions</xsl:with-param>
         <xsl:with-param name="description">
            <xsl:text>(Applies to element declarations). </xsl:text>
            <xsl:text>Prohibits element declarations from nominating themselves as being able to substitute a given element declaration, if they have types that are derived from the original element's type using the specified derivation methods.</xsl:text>
         </xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">TargetNS</xsl:with-param>
         <xsl:with-param name="term">Target Namespace</xsl:with-param>
         <xsl:with-param name="description">The target namespace identifies the namespace that components in this schema belongs to. If no target namespace is provided, then the schema components do not belong to any namespace.</xsl:with-param>
      </xsl:call-template>

      <xsl:call-template name="PrintGlossaryTerm">
         <xsl:with-param name="code">Unique</xsl:with-param>
         <xsl:with-param name="term">Uniqueness Constraint</xsl:with-param>
         <xsl:with-param name="description">Ensures uniqueness of an element/attribute value, or a combination of values, within a specified scope.</xsl:with-param>
         <xsl:with-param name="link">http://www.w3.org/TR/xmlschema-1/#cIdentity-constraint_Definitions</xsl:with-param>
      </xsl:call-template>
   </xsl:template>

  <xsl:template name="PrintGlossaryTerm">
    <xsl:param name="code"/>
    <xsl:param name="term"/>
    <xsl:param name="description"/>
    <xsl:param name="link"/>
    <xsl:text>
</xsl:text>
    <xsl:value-of select="$term"/>
    <xsl:text>
    </xsl:text>
    <xsl:value-of select="$description"/>
    <xsl:if test="$link != ''">
      <xsl:text> See: </xsl:text>
      <xsl:call-template name="PrintURI">
        <xsl:with-param name="uri" select="$link"/>
      </xsl:call-template>
      <xsl:text>.</xsl:text>
    </xsl:if>
  </xsl:template>

   <xsl:template match="xsd:element" mode="hierarchy">
     <xsl:variable name="members">
       <xsl:call-template name="PrintSGroupMembers">
         <xsl:with-param name="element" select="."/>
       </xsl:call-template>
     </xsl:variable>
      <xsl:variable name="hasMembers">
         <xsl:if test="normalize-space($members)!=''">
            <xsl:text>true</xsl:text>
         </xsl:if>
      </xsl:variable>
      <xsl:if test="@substitutionGroup or normalize-space($hasMembers)='true'">
        <xsl:if test="@substitutionGroup">
          <xsl:text>- This element can be used wherever the following element is referenced:</xsl:text>
          <xsl:call-template name="PrintElementRef">
            <xsl:with-param name="ref" select="@substitutionGroup"/>
          </xsl:call-template>
        </xsl:if>
        <xsl:if test="normalize-space($hasMembers)='true'">
          <xsl:text>- The following elements can be used wherever this element is referenced:</xsl:text>
          <xsl:copy-of select="$members"/>
        </xsl:if>
      </xsl:if>
   </xsl:template>

   <xsl:template match="xsd:complexType" mode="hierarchy">
     <xsl:variable name="superTypes">
       <xsl:choose>
         <xsl:when test="xsd:simpleContent or xsd:complexContent">
           <xsl:call-template name="PrintSupertypes">
             <xsl:with-param name="type" select="."/>
           </xsl:call-template>
         </xsl:when>
         <xsl:otherwise>
           <xsl:text>None</xsl:text>
         </xsl:otherwise>
       </xsl:choose>
     </xsl:variable>
     <xsl:text>
+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | </xsl:text>
     <xsl:value-of select="$superTypes"/>
     <xsl:call-template name="PrintHeaderLine">
       <xsl:with-param name="header" select="' '"/>
       <xsl:with-param name="length" select="105 - string-length($superTypes)"/>
     </xsl:call-template>
     <xsl:text> |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | </xsl:text>
     <xsl:call-template name="PrintComplexSubtypes">
       <xsl:with-param name="type" select="."/>
     </xsl:call-template>
     <xsl:text>
+-------------+-----------------------------------------------------------------------------------------------------------+
</xsl:text>
   </xsl:template>

   <xsl:template match="xsd:simpleType" mode="hierarchy">
     <xsl:variable name="superTypes">
       <xsl:choose>
         <xsl:when test="xsd:restriction">
           <xsl:call-template name="PrintSupertypes">
             <xsl:with-param name="type" select="."/>
           </xsl:call-template>
         </xsl:when>
         <xsl:otherwise>
           <xsl:text>None</xsl:text>
         </xsl:otherwise>
       </xsl:choose>
     </xsl:variable>
     <xsl:text>
+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | </xsl:text>
     <xsl:value-of select="$superTypes"/>
     <xsl:call-template name="PrintHeaderLine">
       <xsl:with-param name="header" select="' '"/>
       <xsl:with-param name="length" select="105 - string-length($superTypes)"/>
     </xsl:call-template>
     <xsl:text> |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | </xsl:text>
     <xsl:call-template name="PrintComplexSubtypes">
         <xsl:with-param name="type" select="."/>
       </xsl:call-template>
     <xsl:text>
+-------------+-----------------------------------------------------------------------------------------------------------+
</xsl:text>
   </xsl:template>

   <xsl:template match="*" mode="hierarchy"/>

  <xsl:template name="PrintSGroupMembers">
    <xsl:param name="element"/>
    <xsl:param name="elementList"/>
    <xsl:param name="indent"/>

    <xsl:variable name="elemName" select="normalize-space($element/@name)"/>
    <xsl:choose>
      <xsl:when test="contains($elementList, concat('*', $elemName, '+'))">
        <xsl:text>
</xsl:text>
        <xsl:value-of select="$indent"/>
        <xsl:text>- </xsl:text>
        <xsl:call-template name="HandleError">
          <xsl:with-param name="isTerminating">false</xsl:with-param>
          <xsl:with-param name="errorMsg">
            <xsl:text>Circular element reference to: </xsl:text>
            <xsl:value-of select="$elemName"/>
          </xsl:with-param>
        </xsl:call-template>
      </xsl:when>
      <xsl:otherwise>
        <!-- Get 'block' attribute. -->
        <xsl:variable name="block">
          <xsl:call-template name="PrintBlockSet">
            <xsl:with-param name="EBV">
              <xsl:choose>
                <xsl:when test="$element/@block">
                  <xsl:value-of select="$element/@block"/>
                </xsl:when>
                <xsl:otherwise>
                  <xsl:value-of select="/xsd:schema/@blockDefault"/>
                </xsl:otherwise>
              </xsl:choose>
            </xsl:with-param>
          </xsl:call-template>
        </xsl:variable>

        <xsl:for-each select="/xsd:schema/xsd:element[normalize-space(@substitutionGroup)=$elemName or normalize-space(substring-after(@substitutionGroup, ':'))=$elemName]">
          <xsl:text>
</xsl:text>
          <xsl:value-of select="$indent"/>
          <xsl:text>- </xsl:text>
          <xsl:call-template name="PrintElementRef">
            <xsl:with-param name="name" select="@name"/>
          </xsl:call-template>
          <xsl:if test="not(contains($block, 'substitution'))">
            <xsl:call-template name="PrintSGroupMembers">
              <xsl:with-param name="element" select="."/>
              <xsl:with-param name="elementList" select="concat($elementList, '*', $elemName, '+')"/>
            </xsl:call-template>
          </xsl:if>
        </xsl:for-each>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <xsl:template name="PrintSupertypes">
    <xsl:param name="type"/>
    <xsl:param name="isCallingType">true</xsl:param>
    <xsl:param name="typeList"/>

    <xsl:variable name="typeName" select="$type/@name"/>
    <xsl:choose>
      <!-- Circular type hierarchy -->
      <xsl:when test="contains($typeList, concat('*', $typeName, '+'))">
        <!-- Note: Error message will be written out in the Sample Instance table. -->
        <xsl:text>(There is a circular type reference)</xsl:text>
      </xsl:when>
      <xsl:otherwise>
        <!-- Get base type reference -->
        <xsl:variable name="baseTypeRef">
          <xsl:choose>
            <!-- Complex type definition -->
            <xsl:when test="local-name($type)='complexType'">
              <xsl:choose>
                <xsl:when test="$type/xsd:simpleContent/xsd:extension">
                  <xsl:value-of select="$type/xsd:simpleContent/xsd:extension/@base"/>
                </xsl:when>
                <xsl:when test="$type/xsd:simpleContent/xsd:restriction">
                  <xsl:value-of select="$type/xsd:simpleContent/xsd:restriction/@base"/>
                </xsl:when>
                <xsl:when test="$type/xsd:complexContent/xsd:extension">
                  <xsl:value-of select="$type/xsd:complexContent/xsd:extension/@base"/>
                </xsl:when>
                <xsl:when test="$type/xsd:complexContent/xsd:restriction">
                  <xsl:value-of select="$type/xsd:complexContent/xsd:restriction/@base"/>
                </xsl:when>
              </xsl:choose>
            </xsl:when>
            <!-- Simple type definition -->
            <xsl:when test="local-name($type)='simpleType'">
              <xsl:choose>
                <xsl:when test="$type/xsd:restriction/@base">
                  <xsl:value-of select="$type/xsd:restriction/@base"/>
                </xsl:when>
                <xsl:when test="$type/xsd:restriction/xsd:simpleType">
                  <xsl:text>Local type definition</xsl:text>
                </xsl:when>
              </xsl:choose>
            </xsl:when>
          </xsl:choose>
        </xsl:variable>

        <!-- Get derivation method -->
        <xsl:variable name="derive">
          <!-- Complex type definition -->
          <xsl:choose>
            <xsl:when test="local-name($type)='complexType'">
              <xsl:choose>
                <xsl:when test="$type/xsd:simpleContent/xsd:extension or $type/xsd:complexContent/xsd:extension">
                  <xsl:text>extension</xsl:text>
                </xsl:when>
                <xsl:when test="$type/xsd:simpleContent/xsd:restriction or $type/xsd:complexContent/xsd:restriction">
                  <xsl:text>restriction</xsl:text>
                </xsl:when>
              </xsl:choose>
            </xsl:when>
            <!-- Simple type definition -->
            <xsl:when test="local-name($type)='simpleType'">
              <xsl:text>restriction</xsl:text>
            </xsl:when>
          </xsl:choose>
        </xsl:variable>

        <xsl:choose>
          <xsl:when test="normalize-space($baseTypeRef)='Local type definition'">
            <xsl:value-of select="$baseTypeRef"/>
            <xsl:text> &lt; </xsl:text>
          </xsl:when>
          <xsl:when test="normalize-space($baseTypeRef)!=''">
            <xsl:variable name="baseTypeName">
              <xsl:call-template name="GetRefName">
                <xsl:with-param name="ref" select="$baseTypeRef"/>
              </xsl:call-template>
            </xsl:variable>
            <xsl:variable name="baseType" select="key('type', $baseTypeName)"/>
            <xsl:choose>
              <xsl:when test="$baseType">
                <xsl:call-template name="PrintSupertypes">
                  <xsl:with-param name="type" select="$baseType"/>
                  <xsl:with-param name="isCallingType">false</xsl:with-param>
                  <xsl:with-param name="typeList" select="concat($typeList, '*', $typeName, '+')"/>
                </xsl:call-template>
              </xsl:when>
              <xsl:otherwise>
                <xsl:call-template name="PrintTypeRef">
                  <xsl:with-param name="ref" select="$baseTypeRef"/>
                </xsl:call-template>
              </xsl:otherwise>
            </xsl:choose>
            <xsl:text> &lt; </xsl:text>
          </xsl:when>
        </xsl:choose>
        <xsl:choose>
          <xsl:when test="$isCallingType='true'">
            <xsl:value-of select="$typeName"/>
          </xsl:when>
          <xsl:otherwise>
            <xsl:call-template name="PrintTypeRef">
              <xsl:with-param name="name" select="$typeName"/>
            </xsl:call-template>
          </xsl:otherwise>
        </xsl:choose>
        <xsl:if test="$derive != ''">
          <xsl:text> (by </xsl:text>
          <xsl:value-of select="$derive"/>
          <xsl:text>)</xsl:text>
        </xsl:if>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <xsl:template name="PrintComplexSubtypes">
    <xsl:param name="type"/>
    <xsl:param name="isCallingType">true</xsl:param>
    <xsl:param name="typeList"/>
    <xsl:variable name="typeName" select="normalize-space($type/@name)"/>
    <xsl:choose>
      <xsl:when test="contains($typeList, concat('*', $typeName, '+'))">
      </xsl:when>
      <xsl:otherwise>
        <xsl:variable name="subTypes">
          <xsl:for-each select="/xsd:schema/xsd:complexType/xsd:complexContent/xsd:restriction[normalize-space(@base)=$typeName or normalize-space(substring-after(@base, ':'))=$typeName] | /xsd:schema/xsd:complexType/xsd:complexContent/xsd:extension[normalize-space(@base)=$typeName or normalize-space(substring-after(@base, ':'))=$typeName]">
            <xsl:text>
|             | - </xsl:text>
            <xsl:variable name="subType" select="../.."/>
            <xsl:variable name="typeRef">
              <xsl:call-template name="PrintTypeRef">
                <xsl:with-param name="name" select="$subType/@name"/>
              </xsl:call-template>
              <xsl:text> (by </xsl:text>
              <xsl:value-of select="local-name(.)"/>
              <xsl:text>)</xsl:text>
            </xsl:variable>
            <xsl:value-of select="$typeRef"/>
            <xsl:call-template name="PrintHeaderLine">
              <xsl:with-param name="header" select="' '"/>
              <xsl:with-param name="length" select="104 - string-length($typeRef)"/>
            </xsl:call-template>
            <xsl:text>|</xsl:text>
            <xsl:call-template name="PrintComplexSubtypes">
              <xsl:with-param name="type" select="$subType"/>
              <xsl:with-param name="isCallingType">false</xsl:with-param>
              <xsl:with-param name="typeList" select="concat($typeList, '*', $typeName, '+')"/>
            </xsl:call-template>
          </xsl:for-each>
        </xsl:variable>
        <xsl:choose>
          <xsl:when test="normalize-space($subTypes)!=''">
            <xsl:text>                                                                                                          |</xsl:text>
            <xsl:copy-of select="$subTypes"/>
          </xsl:when>
          <xsl:when test="$isCallingType='true'">
            <xsl:text>None                                                                                                      |</xsl:text>
          </xsl:when>
        </xsl:choose>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

   <xsl:template name="PrintSimpleSubtypes">
      <xsl:param name="type"/>
      <xsl:param name="isCallingType">true</xsl:param>
      <xsl:param name="typeList"/>

      <xsl:variable name="typeName" select="normalize-space($type/@name)"/>
      <xsl:choose>
         <xsl:when test="contains($typeList, concat('*', $typeName, '+'))">
         </xsl:when>
         <xsl:otherwise>
            <xsl:variable name="simpleSubTypes">
              <xsl:text>
</xsl:text>
                  <xsl:for-each select="/xsd:schema/xsd:simpleType/xsd:restriction[normalize-space(@base)=$typeName or normalize-space(substring-after(@base, ':'))=$typeName]">
                    <xsl:text>
- </xsl:text>
                        <xsl:variable name="subType" select=".."/>
                        <!-- Write out type name -->
                        <xsl:call-template name="PrintTypeRef">
                           <xsl:with-param name="name" select="$subType/@name"/>
                        </xsl:call-template>
                        <!-- Write derivation method -->
                        <xsl:text> (by restriction)</xsl:text>
                        <!-- Make recursive call to write sub-types of this sub-type -->
                           <xsl:call-template name="PrintSimpleSubtypes">
                              <xsl:with-param name="type" select="$subType"/>
                              <xsl:with-param name="isCallingType">false</xsl:with-param>
                              <xsl:with-param name="typeList" select="concat($typeList, '*', $typeName, '+')"/>
                           </xsl:call-template>
                  </xsl:for-each>
                     <xsl:text>

</xsl:text>
            </xsl:variable>
            <!-- Find sub-types that are complex type definitions -->
            <xsl:variable name="complexSubTypes">
              <xsl:text>
</xsl:text>
                  <xsl:for-each select="/xsd:schema/xsd:complexType/xsd:simpleContent/xsd:restriction[normalize-space(@base)=$typeName or normalize-space(substring-after(@base, ':'))=$typeName] | /xsd:schema/xsd:complexType/xsd:simpleContent/xsd:extension[normalize-space(@base)=$typeName or normalize-space(substring-after(@base, ':'))=$typeName]">
                    <xsl:text>
- </xsl:text>
                        <xsl:variable name="subType" select="../.."/>
                        <!-- Write out type name -->
                        <xsl:call-template name="PrintTypeRef">
                           <xsl:with-param name="name" select="$subType/@name"/>
                        </xsl:call-template>
                        <!-- Write derivation method -->
                        <xsl:text> (by </xsl:text>
                        <xsl:value-of select="local-name(.)"/>
                        <xsl:text>)</xsl:text>
                        <!-- Make recursive call to write sub-types of this sub-type -->
                           <xsl:call-template name="PrintComplexSubtypes">
                              <xsl:with-param name="type" select="$subType"/>
                              <xsl:with-param name="isCallingType">false</xsl:with-param>
                              <xsl:with-param name="typeList" select="concat($typeList, '*', $typeName, '+')"/>
                           </xsl:call-template>
                  </xsl:for-each>
                    <xsl:text>

</xsl:text>
            </xsl:variable>

            <xsl:variable name="hasSimpleSubTypes">
               <xsl:if test="normalize-space($simpleSubTypes)!=''">
                  <xsl:text>true</xsl:text>
               </xsl:if>
            </xsl:variable>
            <xsl:variable name="hasComplexSubTypes">
               <xsl:if test="normalize-space($complexSubTypes)!=''">
                  <xsl:text>true</xsl:text>
               </xsl:if>
            </xsl:variable>
            <!-- Print out sub types -->
            <xsl:choose>
               <xsl:when test="$hasSimpleSubTypes='true' or $hasComplexSubTypes='true'">
                  <xsl:if test="$hasSimpleSubTypes='true'">
                     <xsl:copy-of select="$simpleSubTypes"/>
                  </xsl:if>
                  <xsl:if test="$hasComplexSubTypes='true'">
                     <xsl:copy-of select="$complexSubTypes"/>
                  </xsl:if>
               </xsl:when>
               <xsl:when test="$isCallingType='true'">
                  <xsl:text>None</xsl:text>
               </xsl:when>
            </xsl:choose>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>


   <xsl:template match="xsd:appinfo | xsd:documentation" mode="properties">
      <xsl:choose>
         <xsl:when test="local-name(.)='appinfo'">
            <xsl:apply-templates select="* | text()" mode="xpp"/>
         </xsl:when>
         <xsl:otherwise>
            <xsl:apply-templates select="* | text()" mode="html"/>
         </xsl:otherwise>
      </xsl:choose>

      <xsl:if test="@source">
         <xsl:if test="* | text()"><br/></xsl:if>
         <xsl:text> More information at: </xsl:text>
         <xsl:call-template name="PrintURI">
            <xsl:with-param name="uri" select="@source"/>
         </xsl:call-template>
         <xsl:text>.</xsl:text>
      </xsl:if>
   </xsl:template>

   <xsl:template match="xsd:attribute" mode="properties">
     <xsl:text>:Name: </xsl:text>
     <xsl:value-of select="@name"/>
     <xsl:text>
</xsl:text>
     <xsl:text>:Type: </xsl:text>
     <xsl:choose>
       <xsl:when test="xsd:simpleType">
         <xsl:text>Locally-defined simple type</xsl:text>
       </xsl:when>
       <xsl:when test="@type">
         <xsl:call-template name="PrintTypeRef">
           <xsl:with-param name="ref" select="@type"/>
         </xsl:call-template>
       </xsl:when>
       <xsl:otherwise>
         <xsl:text>anySimpleType</xsl:text>
       </xsl:otherwise>
     </xsl:choose>
     <xsl:text>
:Multiplicity: </xsl:text>
     <xsl:call-template name="PrintOccurs">
       <xsl:with-param name="component" select="."/>
     </xsl:call-template>
     <xsl:text>
</xsl:text>
     <!-- Default Value -->
     <xsl:if test="@default">
       <xsl:text>:Default Value: </xsl:text>
       <xsl:value-of select="@default"/>
       <xsl:text>
</xsl:text>
       </xsl:if>
       <!-- Fixed Value -->
       <xsl:if test="@fixed">
         <xsl:text>:Fixed Value: </xsl:text>
         <xsl:value-of select="@fixed"/>
         <xsl:text>
</xsl:text>
       </xsl:if>
       <!-- Annotation -->
       <xsl:call-template name="PrintAnnotation">
          <xsl:with-param name="component" select="."/>
       </xsl:call-template>
   </xsl:template>

   <!--
     Prints out Properties table for a top-level
     attribute group or model group definition.
     -->
   <xsl:template match="xsd:attributeGroup | xsd:group" mode="properties">
     <xsl:text>:Name: </xsl:text>
     <xsl:value-of select="@name"/>
     <xsl:text>
</xsl:text>
     <xsl:call-template name="PrintAnnotation">
        <xsl:with-param name="component" select="."/>
     </xsl:call-template>
     <xsl:text>
</xsl:text>
   </xsl:template>

  <xsl:template match="xsd:complexType" mode="properties">
    <xsl:text>:Name: </xsl:text>
    <xsl:value-of select="@name"/>
    <xsl:text>
:</xsl:text>
    <xsl:call-template name="PrintGlossaryTermRef">
      <xsl:with-param name="code">Abstract</xsl:with-param>
      <xsl:with-param name="term">Abstract</xsl:with-param>
    </xsl:call-template>
    <xsl:text>: </xsl:text>
    <xsl:call-template name="PrintBoolean">
      <xsl:with-param name="boolean" select="@abstract"/>
    </xsl:call-template>
    <xsl:text>
</xsl:text>
    <xsl:variable name="final">
      <xsl:call-template name="PrintDerivationSet">
        <xsl:with-param name="EBV">
          <xsl:choose>
            <xsl:when test="@final">
              <xsl:value-of select="@final"/>
            </xsl:when>
            <xsl:otherwise>
              <xsl:value-of select="/xsd:schema/@finalDefault"/>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:with-param>
      </xsl:call-template>
    </xsl:variable>
    <xsl:if test="normalize-space($final)!=''">
      <xsl:text>:</xsl:text>
      <xsl:call-template name="PrintGlossaryTermRef">
        <xsl:with-param name="code">TypeFinal</xsl:with-param>
        <xsl:with-param name="term">Prohibited Derivations</xsl:with-param>
      </xsl:call-template>
      <xsl:text>: </xsl:text>
      <xsl:value-of select="$final"/>
      <xsl:text>
</xsl:text>
    </xsl:if>
    <xsl:variable name="block">
      <xsl:call-template name="PrintDerivationSet">
        <xsl:with-param name="EBV">
          <xsl:choose>
            <xsl:when test="@block">
              <xsl:value-of select="@block"/>
            </xsl:when>
            <xsl:otherwise>
              <xsl:value-of select="/xsd:schema/@blockDefault"/>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:with-param>
      </xsl:call-template>
    </xsl:variable>
    <xsl:if test="normalize-space($block)!=''">
      <xsl:text>:</xsl:text>
      <xsl:call-template name="PrintGlossaryTermRef">
        <xsl:with-param name="code">TypeBlock</xsl:with-param>
        <xsl:with-param name="term">Prohibited Substitutions</xsl:with-param>
      </xsl:call-template>
      <xsl:text>: </xsl:text>
      <xsl:value-of select="$block"/>
      <xsl:text>
</xsl:text>
    </xsl:if>
    <xsl:call-template name="PrintAnnotation">
      <xsl:with-param name="component" select="."/>
    </xsl:call-template>
    <xsl:text>
Properties
""""""""""
</xsl:text>
    <xsl:apply-templates mode="member" select="."/>
  </xsl:template>

  <xsl:template match="xsd:element" mode="member">
    <xsl:text>
Element </xsl:text>
    <xsl:value-of select="@name"/>
    <xsl:text>
</xsl:text>
    <xsl:call-template name="PrintHeaderLine">
      <xsl:with-param name="header" select="'~'"/>
      <xsl:with-param name="length" select="string-length(@name) + 8"/>
    </xsl:call-template>
    <xsl:text>
</xsl:text>
    <xsl:apply-templates mode="properties" select="."/>
  </xsl:template>

  <xsl:template match="xsd:attribute" mode="member">
    <xsl:text>
Attribute </xsl:text>
    <xsl:value-of select="@name"/>
    <xsl:text>
</xsl:text>
    <xsl:call-template name="PrintHeaderLine">
      <xsl:with-param name="header" select="'~'"/>
      <xsl:with-param name="length" select="string-length(@name) + 10"/>
    </xsl:call-template>
    <xsl:text>
</xsl:text>
    <xsl:apply-templates mode="properties" select="."/>    
  </xsl:template>

  <xsl:template match="xsd:annotation" mode="member"/>

  <xsl:template match="xsd:all" mode="member">
    <xsl:text>
The following properties may appear in any order:
</xsl:text>
    <xsl:apply-templates mode="member" select="*"/>
  </xsl:template>

  <xsl:template match="xsd:element" mode="properties">
    <xsl:text>:Name: </xsl:text>
    <xsl:value-of select="@name"/>
    <xsl:text>
:Type: </xsl:text>
    <xsl:choose>
      <xsl:when test="xsd:simpleType">
        <xsl:text>Locally-defined simple type</xsl:text>
      </xsl:when>
      <xsl:when test="xsd:complexType">
        <xsl:text>Locally-defined complex type</xsl:text>
      </xsl:when>
      <xsl:when test="@type">
        <xsl:call-template name="PrintTypeRef">
          <xsl:with-param name="ref" select="@type"/>
        </xsl:call-template>
      </xsl:when>
      <xsl:otherwise>
        <xsl:text>anyType</xsl:text>
      </xsl:otherwise>
    </xsl:choose>
    <xsl:text>
:Multiplicity: </xsl:text>
    <xsl:call-template name="PrintOccurs">
      <xsl:with-param name="component" select="."/>
    </xsl:call-template>
    <xsl:text>
:</xsl:text>
    <xsl:call-template name="PrintGlossaryTermRef">
      <xsl:with-param name="code">Nillable</xsl:with-param>
      <xsl:with-param name="term">Nillable</xsl:with-param>
    </xsl:call-template>
    <xsl:text>: </xsl:text>
    <xsl:call-template name="PrintBoolean">
      <xsl:with-param name="boolean" select="@nillable"/>
    </xsl:call-template>
    <xsl:text>
:</xsl:text>
    <xsl:call-template name="PrintGlossaryTermRef">
      <xsl:with-param name="code">Abstract</xsl:with-param>
      <xsl:with-param name="term">Abstract</xsl:with-param>
    </xsl:call-template>
    <xsl:text>: </xsl:text>
    <xsl:call-template name="PrintBoolean">
      <xsl:with-param name="boolean" select="@abstract"/>
    </xsl:call-template>
    <xsl:text>
</xsl:text>
    <xsl:if test="@default">
      <xsl:text>:Default Value: </xsl:text>
      <xsl:value-of select="@default"/>
      <xsl:text>
</xsl:text>
    </xsl:if>
    <xsl:if test="@fixed">
      <xsl:text>:Fixed Value: </xsl:text>
      <xsl:value-of select="@fixed"/>
      <xsl:text>
</xsl:text>
    </xsl:if>
    <xsl:variable name="final">
      <xsl:call-template name="PrintDerivationSet">
        <xsl:with-param name="EBV">
          <xsl:choose>
            <xsl:when test="@final">
              <xsl:value-of select="@final"/>
            </xsl:when>
            <xsl:otherwise>
              <xsl:value-of select="/xsd:schema/@finalDefault"/>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:with-param>
      </xsl:call-template>
    </xsl:variable>
    <xsl:if test="normalize-space($final)!=''">
      <xsl:text>:</xsl:text>
      <xsl:call-template name="PrintGlossaryTermRef">
        <xsl:with-param name="code">ElemFinal</xsl:with-param>
        <xsl:with-param name="term">Substitution Group Exclusions</xsl:with-param>
      </xsl:call-template>
      <xsl:text>: </xsl:text>
      <xsl:value-of select="$final"/>
      <xsl:text>
</xsl:text>
    </xsl:if>
    <xsl:variable name="block">
      <xsl:call-template name="PrintBlockSet">
        <xsl:with-param name="EBV">
          <xsl:choose>
            <xsl:when test="@block">
              <xsl:value-of select="@block"/>
            </xsl:when>
            <xsl:otherwise>
              <xsl:value-of select="/xsd:schema/@blockDefault"/>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:with-param>
      </xsl:call-template>
    </xsl:variable>
    <xsl:if test="normalize-space($block)!=''">
      <xsl:text>:</xsl:text>
      <xsl:call-template name="PrintGlossaryTermRef">
        <xsl:with-param name="code">ElemBlock</xsl:with-param>
        <xsl:with-param name="term">Disallowed Substitutions</xsl:with-param>
      </xsl:call-template>
      <xsl:text>: </xsl:text>
      <xsl:value-of select="$block"/>
      <xsl:text>
</xsl:text>
    </xsl:if>
    <!-- Annotation -->
    <xsl:call-template name="PrintAnnotation">
      <xsl:with-param name="component" select="."/>
    </xsl:call-template>
  </xsl:template>

   <!--
     Prints out the Properties table for the root
     schema element.
     -->
  <xsl:template match="xsd:schema" mode="properties">
    <xsl:text>:</xsl:text>
    <xsl:call-template name="PrintGlossaryTermRef">
      <xsl:with-param name="code">TargetNS</xsl:with-param>
      <xsl:with-param name="term">Target Namespace</xsl:with-param>
    </xsl:call-template>
    <xsl:text>: </xsl:text>
    <xsl:choose>
      <xsl:when test="@targetNamespace">
        <xsl:value-of select="@targetNamespace"/>
      </xsl:when>
      <xsl:otherwise>
        <xsl:text>None</xsl:text>
      </xsl:otherwise>
    </xsl:choose>
    <xsl:text>
</xsl:text>
    <xsl:if test="@version">
      <xsl:text>:Version: </xsl:text>
      <xsl:value-of select="@version"/>
      <xsl:text>
</xsl:text>
    </xsl:if>
    <xsl:if test="@xml:lang">
      <xsl:text>:Language: </xsl:text>
      <xsl:value-of select="@xml:lang"/>
      <xsl:text>
</xsl:text>
    </xsl:if>
    <xsl:text>:Element Qualifiers: </xsl:text>
    <xsl:choose>
      <xsl:when test="normalize-space(@elementFormDefault)='qualified'">
        <xsl:text>By default, local element declarations belong to this schema's target namespace.</xsl:text>
      </xsl:when>
      <xsl:otherwise>
        <xsl:text>By default, local element declarations have no namespace.</xsl:text>
      </xsl:otherwise>
    </xsl:choose>
    <xsl:text>
:Attribute Qualifiers: </xsl:text>
    <xsl:choose>
      <xsl:when test="normalize-space(@attributeFormDefault)='qualified'">
        <xsl:text>By default, local attribute declarations belong to this schema's target namespace.</xsl:text>
      </xsl:when>
      <xsl:otherwise>
        <xsl:text>By default, local attribute declarations have no namespace.</xsl:text>
      </xsl:otherwise>
    </xsl:choose>
    <xsl:text>
</xsl:text>
    <xsl:if test="xsd:include or xsd:import or xsd:redefine">
      <xsl:text>:Imports: </xsl:text>
      <xsl:if test="xsd:import">
        <xsl:text>This schema imports schema(s) from the following namespace(s):</xsl:text>
        <xsl:for-each select="xsd:import">
          <xsl:text>
  - </xsl:text>
          <xsl:value-of select="@namespace"/>
        </xsl:for-each>
        <xsl:text>
</xsl:text>
      </xsl:if>
    </xsl:if>
    <!-- Annotation -->
    <xsl:call-template name="PrintAnnotation">
      <xsl:with-param name="component" select="."/>
    </xsl:call-template>
  </xsl:template>

   <xsl:template match="xsd:simpleType" mode="properties">
     <xsl:text>:Name: </xsl:text>
     <xsl:value-of select="@name"/>
     <xsl:text>
:Content: </xsl:text>
     <xsl:call-template name="PrintSimpleConstraints">
                  <xsl:with-param name="simpleContent" select="."/>
               </xsl:call-template>
         <xsl:variable name="final">
            <xsl:call-template name="PrintSimpleDerivationSet">
               <xsl:with-param name="EBV">
                  <xsl:choose>
                     <xsl:when test="@final">
                        <xsl:value-of select="@final"/>
                     </xsl:when>
                     <xsl:otherwise>
                        <xsl:value-of select="/xsd:schema/@finalDefault"/>
                     </xsl:otherwise>
                  </xsl:choose>
               </xsl:with-param>
            </xsl:call-template>
         </xsl:variable>
         <xsl:if test="normalize-space($final)!=''">
           <xsl:text>
:</xsl:text>
                  <xsl:call-template name="PrintGlossaryTermRef">
                     <xsl:with-param name="code">TypeFinal</xsl:with-param>
                     <xsl:with-param name="term">Prohibited Derivations</xsl:with-param>
                  </xsl:call-template>
           <xsl:text>: </xsl:text>
           <xsl:value-of select="$final"/>
         </xsl:if>
     <xsl:text>
</xsl:text>
         <!-- Annotation -->
         <xsl:call-template name="PrintAnnotation">
            <xsl:with-param name="component" select="."/>
         </xsl:call-template>

     <xsl:choose>
       <xsl:when test="xsd:restriction/xsd:enumeration">
         <xsl:text>
Members
"""""""
</xsl:text>
         <xsl:apply-templates mode="properties" select="xsd:restriction/xsd:enumeration"/>
       </xsl:when>
     </xsl:choose>
   </xsl:template>

  <xsl:template match="xsd:enumeration" mode="properties">
    <xsl:text>* </xsl:text>
    <xsl:value-of select="@value"/>
    <xsl:choose>
      <xsl:when test="xsd:annotation/xsd:documentation">
        <xsl:text>: </xsl:text>
        <xsl:value-of select="xsd:annotation/xsd:documentation"/>
      </xsl:when>
    </xsl:choose>
    <xsl:text>
</xsl:text>
  </xsl:template>

   <xsl:template match="*" mode="properties"/>

  <xsl:template name="PrintAnnotation">
    <xsl:param name="component"/>

    <xsl:if test="$component/xsd:annotation/xsd:documentation">
      <xsl:text>:Documentation: </xsl:text>
      <xsl:for-each select="$component/xsd:annotation/xsd:documentation">
        <xsl:text>
  </xsl:text>
        <xsl:apply-templates select="." mode="properties"/>
      </xsl:for-each>
      <xsl:text>
</xsl:text>
    </xsl:if>
    <xsl:if test="$component/xsd:annotation/xsd:appinfo">
      <xsl:text>:Application Data: </xsl:text>
      <xsl:for-each select="$component/xsd:annotation/xsd:appinfo">
        <xsl:text>
  </xsl:text>
        <xsl:apply-templates select="." mode="properties"/>
      </xsl:for-each>
      <xsl:text>
</xsl:text>
    </xsl:if>
  </xsl:template>

  <xsl:template name="PrintSimpleConstraints">
    <xsl:param name="simpleContent"/>
    <xsl:param name="typeList"/>

    <xsl:choose>
      <!-- Derivation by restriction -->
      <xsl:when test="$simpleContent/xsd:restriction">
        <xsl:call-template name="PrintSimpleRestriction">
          <xsl:with-param name="restriction" select="$simpleContent/xsd:restriction"/>
          <xsl:with-param name="typeList" select="$typeList"/>
        </xsl:call-template>
      </xsl:when>
      <!-- Derivation by list -->
      <xsl:when test="$simpleContent/xsd:list">
        <xsl:text>List of: </xsl:text>
        <xsl:choose>
          <!-- Globally-defined item type -->
          <xsl:when test="$simpleContent/xsd:list/@itemType">
            <xsl:call-template name="PrintTypeRef">
              <xsl:with-param name="ref" select="$simpleContent/xsd:list/@itemType"/>
            </xsl:call-template>
          </xsl:when>
          <!-- Locally-defined item type -->
          <xsl:otherwise>
            <xsl:text>Locally defined type:</xsl:text>
            <xsl:call-template name="PrintSimpleConstraints">
              <xsl:with-param name="simpleContent" select="$simpleContent/xsd:list/xsd:simpleType"/>
              <xsl:with-param name="typeList" select="$typeList"/>
            </xsl:call-template>
          </xsl:otherwise>
        </xsl:choose>
      </xsl:when>
      <!-- Derivation by union -->
      <xsl:when test="$simpleContent/xsd:union">
        <xsl:text>Union of following types: </xsl:text>
        <!-- Globally-defined member types -->
        <xsl:if test="$simpleContent/xsd:union/@memberTypes">
          <xsl:call-template name="PrintWhitespaceList">
            <xsl:with-param name="value" select="$simpleContent/xsd:union/@memberTypes"/>
            <xsl:with-param name="compType">type</xsl:with-param>
            <xsl:with-param name="isInList">true</xsl:with-param>
          </xsl:call-template>
        </xsl:if>
        <!-- Locally-defined member types -->
        <xsl:for-each select="$simpleContent/xsd:union/xsd:simpleType">
          <xsl:text>
  - </xsl:text>
          <xsl:text>Locally defined type:</xsl:text>
          <xsl:call-template name="PrintSimpleConstraints">
            <xsl:with-param name="simpleContent" select="."/>
            <xsl:with-param name="typeList" select="$typeList"/>
          </xsl:call-template>
        </xsl:for-each>
      </xsl:when>
    </xsl:choose>
  </xsl:template>

   <xsl:template name="PrintSimpleRestriction">
      <xsl:param name="restriction"/>
      <xsl:param name="typeList"/>

      <xsl:variable name="typeName" select="$restriction/parent::xsd:simpleType/@name"/>

      <xsl:choose>
         <!-- Circular type hierarchy -->
         <xsl:when test="$typeName != '' and contains($typeList, concat('*', $typeName, '+'))">
           <xsl:text>
  - </xsl:text>
               <xsl:call-template name="HandleError">
                  <xsl:with-param name="isTerminating">false</xsl:with-param>
                  <xsl:with-param name="errorMsg">
                     <xsl:text>Circular type reference to '</xsl:text>
                     <xsl:value-of select="$typeName"/>
                     <xsl:text>' in type hierarchy.</xsl:text>
                  </xsl:with-param>
               </xsl:call-template>
         </xsl:when>
         <!-- Locally-defined base type -->
         <xsl:when test="$restriction/xsd:simpleType">
            <xsl:call-template name="PrintSimpleConstraints">
               <xsl:with-param name="simpleContent" select="$restriction/xsd:simpleType"/>
               <xsl:with-param name="typeList" select="$typeList"/>
            </xsl:call-template>
         </xsl:when>
         <!-- Base type reference -->
         <xsl:when test="$restriction">
            <xsl:variable name="baseTypeRef" select="$restriction/@base"/>
            <xsl:variable name="baseTypeName">
               <xsl:call-template name="GetRefName">
                  <xsl:with-param name="ref" select="$baseTypeRef"/>
               </xsl:call-template>
            </xsl:variable>
            <xsl:variable name="baseTypeNS">
               <xsl:call-template name="GetRefNS">
                  <xsl:with-param name="ref" select="$baseTypeRef"/>
               </xsl:call-template>
            </xsl:variable>

            <xsl:choose>
               <!-- Base type is built-in XSD type -->
               <xsl:when test="$baseTypeNS=$XSD_NS">
                     <xsl:text>Base XSD Type: </xsl:text>
                     <xsl:choose>
                        <xsl:when test="normalize-space(/xsd:schema/@targetNamespace)=$XSD_NS">
                           <xsl:call-template name="PrintTypeRef">
                              <xsl:with-param name="ref" select="$baseTypeRef"/>
                           </xsl:call-template>
                        </xsl:when>
                        <xsl:otherwise>
                           <xsl:value-of select="$baseTypeName"/>
                        </xsl:otherwise>
                     </xsl:choose>
               </xsl:when>
               <!-- Other types -->
               <xsl:otherwise>
                  <xsl:variable name="baseType" select="key('simpleType', $baseTypeName)"/>
                  <xsl:choose>
                     <!-- Base type found -->
                     <xsl:when test="$baseType">
                        <xsl:call-template name="PrintSimpleConstraints">
                           <xsl:with-param name="simpleContent" select="$baseType"/>
                           <xsl:with-param name="typeList" select="concat($typeList, '*', $typeName, '+')"/>
                        </xsl:call-template>
                     </xsl:when>
                     <!-- Base type not found -->
                     <xsl:otherwise>
                           <xsl:text>'</xsl:text>
                           <xsl:value-of select="$baseTypeName"/>
                           <xsl:text>' super type was not found in this schema. </xsl:text>
                           <xsl:text>Its facets could not be printed out.</xsl:text>
                     </xsl:otherwise>
                  </xsl:choose>
               </xsl:otherwise>
            </xsl:choose>
         </xsl:when>
      </xsl:choose>

      <xsl:variable name="enumeration">
         <xsl:call-template name="PrintEnumFacets">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
      </xsl:variable>
      <xsl:variable name="pattern">
         <xsl:call-template name="PrintPatternFacet">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
      </xsl:variable>
      <xsl:variable name="range">
         <xsl:call-template name="PrintRangeFacets">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
      </xsl:variable>
      <xsl:variable name="totalDigits">
         <xsl:call-template name="PrintTotalDigitsFacet">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
      </xsl:variable>
      <xsl:variable name="fractionDigits">
         <xsl:call-template name="PrintFractionDigitsFacet">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
      </xsl:variable>
      <xsl:variable name="length">
         <xsl:call-template name="PrintLengthFacets">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
      </xsl:variable>
      <xsl:variable name="whitespace">
         <xsl:call-template name="PrintWhitespaceFacet">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
      </xsl:variable>

      <xsl:if test="$enumeration!='' or $pattern!='' or $range!='' or $totalDigits!='' or $fractionDigits!='' or $length!='' or $whitespace!=''">
            <xsl:if test="$enumeration!=''">
                  <xsl:copy-of select="$enumeration"/>
            </xsl:if>
            <xsl:if test="$pattern!=''">
                  <xsl:copy-of select="$pattern"/>
            </xsl:if>
            <xsl:if test="$range!=''">
                  <xsl:copy-of select="$range"/>
            </xsl:if>
            <xsl:if test="$totalDigits!=''">
                  <xsl:copy-of select="$totalDigits"/>
            </xsl:if>
            <xsl:if test="$fractionDigits!=''">
                  <xsl:copy-of select="$fractionDigits"/>
            </xsl:if>
            <xsl:if test="$length!=''">
                  <xsl:copy-of select="$length"/>
            </xsl:if>
            <xsl:if test="$whitespace!=''">
                  <xsl:copy-of select="$whitespace"/>
            </xsl:if>
      </xsl:if>
   </xsl:template>

   <xsl:template name="SampleInstanceTable">
      <xsl:param name="component"/>
      <xsl:if test="local-name($component)!='simpleType' and local-name($component)!='notation'">
         <xsl:variable name="componentID">
            <xsl:call-template name="GetComponentID">
               <xsl:with-param name="component" select="$component"/>
            </xsl:call-template>
         </xsl:variable>

         <xsl:call-template name="CollapseableBox">
            <xsl:with-param name="id" select="concat($componentID, '_xibox')"/>
            <xsl:with-param name="caption">XML Instance Representation</xsl:with-param>
            <xsl:with-param name="contents">
               <xsl:apply-templates select="$component" mode="sample"/>
            </xsl:with-param>
         </xsl:call-template>
      </xsl:if>
   </xsl:template>

   <xsl:template match="xsd:all" mode="sample">
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>

      <xsl:if test="normalize-space(@maxOccurs)!='0'">
            <!-- Documentation -->
            <xsl:call-template name="PrintSampleDocumentation">
               <xsl:with-param name="component" select="."/>
            </xsl:call-template>

         <!-- Content -->
         <xsl:apply-templates select="xsd:*" mode="sample">
            <xsl:with-param name="isInherited" select="$isInherited"/>
            <xsl:with-param name="isNewField" select="$isNewField"/>
            <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            <xsl:with-param name="typeList" select="$typeList"/>
         </xsl:apply-templates>
      </xsl:if>
   </xsl:template>

   <!--
     Prints out a sample XML instance representation of an element 
     content wild card.
     Param(s):
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
     -->
   <xsl:template match="xsd:any | xsd:anyAttribute" mode="sample">
      <xsl:param name="margin">0</xsl:param>

      <div class="other" style="margin-left: {$margin}em">
         <xsl:call-template name="PrintWildcard">
            <xsl:with-param name="componentType">
               <xsl:choose>
                  <xsl:when test="local-name(.)='anyAttribute'">attribute</xsl:when>
                  <xsl:otherwise>element</xsl:otherwise>
               </xsl:choose>
            </xsl:with-param>
            <xsl:with-param name="namespace" select="@namespace"/>
            <xsl:with-param name="processContents" select="@processContents"/>
            <xsl:with-param name="minOccurs" select="@minOccurs"/>
            <xsl:with-param name="maxOccurs" select="@maxOccurs"/>
         </xsl:call-template>
      </div>
   </xsl:template>

   <!--
     Prints out a sample XML instance representation 
     of an attribute declaration.
     Param(s):
            subTypeAttrs (String) optional
                List of attributes in sub-types of the type that 
                contains this attribute
            isInherited (boolean) optional
                If true, display attribute using 'inherited' CSS 
                class.
            isNewField (boolean) optional
                If true, display attribute using 'newFields' CSS 
                class.
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            addBR (boolean) optional
                If true, add <br/> before attribute.
            schemaLoc (String) optional
                Schema file containing this attribute declaration;
                if in current schema, 'schemaLoc' is set to 'this'.
     -->
   <xsl:template match="xsd:attribute[@name]" mode="sample">
      <xsl:param name="subTypeAttrs"/>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="addBR">false</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>

      <!-- Get attribute namespace -->
      <xsl:variable name="attrNS">
         <xsl:call-template name="GetAttributeNS">
            <xsl:with-param name="attribute" select="."/>
         </xsl:call-template>
      </xsl:variable>

      <xsl:choose>
         <xsl:when test="contains($subTypeAttrs, concat('*', normalize-space($attrNS), '+', normalize-space(@name), '+'))">
            <!-- IGNORE: Sub type has attribute with same name; 
                 Sub-type's attribute declaration will override this 
                 one. -->
         </xsl:when>
         <xsl:when test="@use and normalize-space(@use)='prohibited'">
            <!-- IGNORE: Attribute is prohibited. -->
         </xsl:when>
         <xsl:otherwise>
            <xsl:if test="$addBR!='false'"><br/></xsl:if>
               <xsl:choose>
                  <xsl:when test="$isNewField!='false'">
                     <xsl:attribute name="class">newFields</xsl:attribute>
                  </xsl:when>
                  <xsl:when test="$isInherited!='false'">
                     <xsl:attribute name="class">inherited</xsl:attribute>
                  </xsl:when>
               </xsl:choose>

               <xsl:text> </xsl:text>
               <xsl:variable name="prefix">
                  <xsl:call-template name="GetAttributePrefix">
                     <xsl:with-param name="attribute" select="."/>
                  </xsl:call-template>
               </xsl:variable>
               <xsl:value-of select="@name"/>
               <xsl:text>="</xsl:text>

               <xsl:choose>
                  <!-- Fixed value is provided -->
                  <xsl:when test="@fixed">
                        <xsl:value-of select="@fixed"/>
                  </xsl:when>
                  <!-- Type reference is provided -->
                  <xsl:when test="@type">
                     <xsl:call-template name="PrintTypeRef">
                        <xsl:with-param name="ref" select="@type"/>
                        <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     </xsl:call-template>
                  </xsl:when>
                  <!-- Local type definition is provided -->
                  <xsl:when test="xsd:simpleType">
                     <xsl:apply-templates select="xsd:simpleType" mode="sample">
                        <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     </xsl:apply-templates>
                  </xsl:when>
                  <xsl:otherwise>
                     anySimpleType
                  </xsl:otherwise>
               </xsl:choose>

               <!-- Don't print occurrence info and documentation
                    for global attributes. -->
               <xsl:if test="local-name(..)!='schema'">
                  <!-- Occurrence info-->
                  <xsl:text> </xsl:text>
                  <xsl:call-template name="PrintOccurs">
                     <xsl:with-param name="component" select="."/>
                  </xsl:call-template>
                  <!-- Documentation -->
                  <xsl:call-template name="PrintSampleDocumentation">
                     <xsl:with-param name="component" select="."/>
                  </xsl:call-template>
               </xsl:if>

               <xsl:text>"</xsl:text>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Prints out a sample XML instance representation 
     of an attribute reference.
     Param(s):
            subTypeAttrs (String) optional
                List of attribute in sub-types of the type that 
                contains this attribute
            isInherited (boolean) optional
                If true, display attributes using 'inherited' CSS 
                class.
            isNewField (boolean) optional
                If true, display attributes using 'newFields' CSS 
                class.
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            addBR (boolean) optional
                If true, add <br/> before attribute.
            schemaLoc (String) optional
                Schema file containing this attribute reference;
                if in current schema, 'schemaLoc' is set to 'this'
     -->
   <xsl:template match="xsd:attribute[@ref]" mode="sample">
      <xsl:param name="subTypeAttrs"/>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="addBR">false</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>

      <!-- Get attribute name -->
      <xsl:variable name="attrName">
         <xsl:call-template name="GetRefName">
            <xsl:with-param name="ref" select="@ref"/>
         </xsl:call-template>
      </xsl:variable>

      <!-- Get attribute namespace -->
      <xsl:variable name="attrNS">
         <xsl:call-template name="GetAttributeNS">
            <xsl:with-param name="attribute" select="."/>
         </xsl:call-template>
      </xsl:variable>

      <xsl:choose>
         <xsl:when test="contains($subTypeAttrs, concat('*', normalize-space($attrNS), '+', normalize-space($attrName), '+'))">
            <!-- IGNORE: Sub type has attribute with same name; 
                 Sub-type's attribute declaration will override this 
                 one. -->
         </xsl:when>
         <xsl:when test="@use and normalize-space(@use)='prohibited'">
            <!-- IGNORE: Attribute is prohibited. -->
         </xsl:when>
         <xsl:otherwise>
            <xsl:if test="$addBR!='false'"><br/></xsl:if>
               <xsl:choose>
                  <xsl:when test="$isNewField!='false'">
                     <xsl:attribute name="class">newFields</xsl:attribute>
                  </xsl:when>
                  <xsl:when test="$isInherited!='false'">
                     <xsl:attribute name="class">inherited</xsl:attribute>
                  </xsl:when>
               </xsl:choose>

               <xsl:text> </xsl:text>
               <xsl:call-template name="PrintAttributeRef">
                  <xsl:with-param name="ref" select="@ref"/>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:call-template>
               <xsl:text>="</xsl:text>
               <!-- Fixed value is provided -->
               <xsl:if test="@fixed">
                     <xsl:value-of select="@fixed"/>
                  <xsl:text> </xsl:text>
               </xsl:if>
               <!-- Print occurs info-->
               <xsl:call-template name="PrintOccurs">
                  <xsl:with-param name="component" select="."/>
               </xsl:call-template>
               <!-- Documentation -->
               <xsl:call-template name="PrintSampleDocumentation">
                  <xsl:with-param name="component" select="."/>
               </xsl:call-template>

               <xsl:text>"</xsl:text>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Prints out a sample XML instance representation of an attribute 
     group definition.
     Param(s):
            schemaLoc (String) optional
                Schema file containing this attribute group 
                definition; if in current schema, 'schemaLoc' is 
                set to 'this'.
     -->
   <xsl:template match="xsd:attributeGroup[@name]" mode="sample">
      <xsl:param name="schemaLoc">this</xsl:param>

      <xsl:for-each select="xsd:attribute | xsd:attributeGroup | xsd:anyAttribute">
         <xsl:variable name="addBR">
            <xsl:choose>
               <xsl:when test="position()!=1">
                  <xsl:text>true</xsl:text>
               </xsl:when>
               <xsl:otherwise>
                  <xsl:text>false</xsl:text>
               </xsl:otherwise>
            </xsl:choose>
         </xsl:variable>

         <xsl:apply-templates select="." mode="sample">
            <xsl:with-param name="addBR" select="$addBR"/>
            <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
         </xsl:apply-templates>
      </xsl:for-each>
   </xsl:template>

   <!--
     Prints out a sample XML instance representation of an attribute 
     group reference.
     Param(s):
            subTypeAttrs (String) optional
                List of attributes in sub-types of the type that 
                contains this attribute group
            isInherited (boolean) optional
                If true, display attributes using 'inherited' CSS 
                class.
            isNewField (boolean) optional
                If true, display attributes using 'newFields' CSS 
                class.
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            parentGroups (String) optional
                List of parent attribute group definitions that 
                contain this attribute group. Used to prevent 
                infinite loops when displaying attribute group 
                definitions. In such a case, writes out an error 
                message and stops processing.
            schemaLoc (String) optional
                Schema file containing this attribute group 
                reference if in current schema, 'schemaLoc' is 
                set to 'this'.
     -->
   <xsl:template match="xsd:attributeGroup[@ref]" mode="sample">
      <xsl:param name="subTypeAttrs"/>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="parentGroups"/>
      <xsl:param name="schemaLoc">this</xsl:param>

      <!-- Get attribute group name -->
      <xsl:variable name="attrGrpName">
         <xsl:call-template name="GetRefName">
            <xsl:with-param name="ref" select="@ref"/>
         </xsl:call-template>
      </xsl:variable>

      <xsl:choose>
         <xsl:when test="contains($parentGroups, concat('*', normalize-space($attrGrpName), '+'))">
            <!-- Circular attribute group definition -->
            <xsl:call-template name="HandleError">
               <xsl:with-param name="isTerminating">false</xsl:with-param>
               <xsl:with-param name="errorMsg">
                  <xsl:text>Circular attribute group reference: </xsl:text>
                  <xsl:value-of select="$attrGrpName"/>
               </xsl:with-param>
            </xsl:call-template>
         </xsl:when>
         <xsl:otherwise>
            <!-- Look for attribute group definition -->
            <xsl:variable name="defLoc">
               <xsl:call-template name="FindComponent">
                  <xsl:with-param name="ref" select="@ref"/>
                  <xsl:with-param name="compType">attribute group</xsl:with-param>
               </xsl:call-template>
            </xsl:variable>

            <xsl:choose>
               <!-- Not found -->
               <xsl:when test="normalize-space($defLoc)='' or normalize-space($defLoc)='none' or normalize-space($defLoc)='xml' or normalize-space($defLoc)='xsd'">
                  <div class="other" style="margin-left: {$margin}em">
                     <xsl:text>Attribute group reference (not shown): </xsl:text>
                     <xsl:call-template name="PrintAttributeGroupRef">
                        <xsl:with-param name="ref" select="@ref"/>
                        <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     </xsl:call-template>
                     <!-- Documentation -->
                     <xsl:call-template name="PrintSampleDocumentation">
                        <xsl:with-param name="component" select="."/>
                     </xsl:call-template>
                  </div>
               </xsl:when>
               <!-- Found in current schema -->
               <xsl:when test="normalize-space($defLoc)='this'">
                  <xsl:variable name="attrGrpDef" select="key('attributeGroup', $attrGrpName)"/>
                  <xsl:apply-templates select="$attrGrpDef/xsd:attribute | $attrGrpDef/xsd:attributeGroup" mode="sample">
                     <xsl:with-param name="subTypeAttrs" select="$subTypeAttrs"/>
                     <xsl:with-param name="isInherited" select="$isInherited"/>
                     <xsl:with-param name="isNewField" select="$isNewField"/>
                     <xsl:with-param name="parentGroups" select="concat($parentGroups, concat('*', normalize-space($attrGrpName), '+'))"/>
                     <xsl:with-param name="margin" select="$margin"/>
                     <xsl:with-param name="addBR">true</xsl:with-param>
                     <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                  </xsl:apply-templates>
               </xsl:when>
               <!-- Found in external schema -->
               <xsl:otherwise>
                  <xsl:variable name="attrGrpDef" select="document($defLoc)/xsd:schema/xsd:attributeGroup[@name=$attrGrpName]"/>
                  <xsl:apply-templates select="$attrGrpDef/xsd:attribute | $attrGrpDef/xsd:attributeGroup" mode="sample">
                     <xsl:with-param name="subTypeAttrs" select="$subTypeAttrs"/>
                     <xsl:with-param name="isInherited" select="$isInherited"/>
                     <xsl:with-param name="isNewField" select="$isNewField"/>
                     <xsl:with-param name="parentGroups" select="concat($parentGroups, concat('*', normalize-space($attrGrpName), '+'))"/>
                     <xsl:with-param name="margin" select="$margin"/>
                     <xsl:with-param name="addBR">true</xsl:with-param>
                     <xsl:with-param name="schemaLoc" select="$defLoc"/>
                  </xsl:apply-templates>
               </xsl:otherwise>
            </xsl:choose>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Prints out a sample XML instance representation of a 'choice' 
     model group.
     Param(s):
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            isInherited (boolean) optional
                If true, display elements using'inherited' CSS class.
            isNewField (boolean) optional
                If true, display elements using 'newFields' CSS class.
            parentGroups (String) optional
                List of parent model group definitions that contain this 
                model group. Used to prevent infinite loops when 
                displaying model group definitions.
            schemaLoc (String) optional
                Schema file containing this choice model group;
                if in current schema, 'schemaLoc' is set to 'this'
            typeList (String) optional
                List of types in this call chain. Name of type starts
                with '*', and ends with '+'. (Used to prevent infinite
                recursive loop.)
     -->
   <xsl:template match="xsd:choice" mode="sample">
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="parentGroups"/>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>

      <xsl:if test="normalize-space(@maxOccurs)!='0'">
            <!-- Documentation -->
            <xsl:call-template name="PrintSampleDocumentation">
               <xsl:with-param name="component" select="."/>
            </xsl:call-template>

         <!-- Content -->
         <xsl:apply-templates select="xsd:*" mode="sample">
            <xsl:with-param name="isInherited" select="$isInherited"/>
            <xsl:with-param name="isNewField" select="$isNewField"/>
            <xsl:with-param name="parentGroups" select="$parentGroups"/>
            <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            <xsl:with-param name="typeList" select="$typeList"/>
         </xsl:apply-templates>
      </xsl:if>
   </xsl:template>

   <!--
     Prints out a sample XML instance from a complex type definition.
     Param(s):
            schemaLoc (String) optional
                Schema file containing this complex type definition; 
                if in current schema, 'schemaLoc' is set to 'this'.
     -->
   <xsl:template match="xsd:complexType" mode="sample">
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="parentGroups"/>

      <xsl:call-template name="PrintSampleComplexElement">
         <xsl:with-param name="type" select="."/>
         <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
	 <xsl:with-param name="parentGroups" select="$parentGroups"/>
      </xsl:call-template>
   </xsl:template>

   <!--
     Prints out a sample XML instance from an element declaration.
     Param(s):
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            isInherited (boolean) optional
                If true, display elements using 'inherited' CSS class.
            isNewField (boolean) optional
                If true, display elements using 'newFields' CSS class.
            schemaLoc (String) optional
                Schema file containing this element declaration; 
                if in current schema, 'schemaLoc' is set to 'this'.
            typeList (String) optional
                List of types in this call chain. Name of type starts
                with '*', and ends with '+'. (Used to prevent infinite
                recursive loop.)
     -->
   <xsl:template match="xsd:element[@name]" mode="sample">
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>
      <xsl:param name="parentGroups"/>

      <xsl:choose>
         <!-- Prohibited element declaration -->
         <xsl:when test="normalize-space(@maxOccurs)='0'">
            <!-- IGNORE if max occurs is zero -->
         </xsl:when>
         <!-- Global element declaration -->
         <xsl:when test="local-name(..)='schema'">
            <xsl:choose>
               <!-- With type reference -->
               <xsl:when test="@type">
                  <xsl:variable name="elemTypeName">
                     <xsl:call-template name="GetRefName">
                        <xsl:with-param name="ref" select="@type"/>
                     </xsl:call-template>
                  </xsl:variable>

                  <!-- Look for complex type definition -->
                  <xsl:variable name="defLoc">
                     <xsl:call-template name="FindComponent">
                        <xsl:with-param name="ref" select="@type"/>
                        <xsl:with-param name="compType">complex type</xsl:with-param>
                     </xsl:call-template>
                  </xsl:variable>

                  <xsl:choose>
                     <!-- Complex type was found in current 
                          schema. -->
                     <xsl:when test="normalize-space($defLoc)='this'">
                        <xsl:variable name="ctype" select="key('complexType', $elemTypeName)"/>
                        <xsl:call-template name="PrintSampleComplexElement">
                           <xsl:with-param name="element" select="."/>
                           <xsl:with-param name="type" select="$ctype"/>
			   <xsl:with-param name="parentGroups" select="$parentGroups"/>
                        </xsl:call-template>
                     </xsl:when>
                     <!-- Complex type was not found. -->
                     <xsl:when test="normalize-space($defLoc)='' or normalize-space($defLoc)='none' or normalize-space($defLoc)='xml' or normalize-space($defLoc)='xsd'">
                        <xsl:call-template name="PrintSampleSimpleElement">
                           <xsl:with-param name="element" select="."/>
                           <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                        </xsl:call-template>
                     </xsl:when>
                     <!-- Complex type was found in external
                          schema. -->
                     <xsl:otherwise>
                        <xsl:variable name="ctype" select="document($defLoc)/xsd:schema/xsd:complexType[@name=$elemTypeName]"/>
                        <xsl:call-template name="PrintSampleComplexElement">
                           <xsl:with-param name="element" select="."/>
                           <xsl:with-param name="type" select="$ctype"/>
			   <xsl:with-param name="parentGroups" select="$parentGroups"/>
                        </xsl:call-template>
                     </xsl:otherwise>
                  </xsl:choose>
               </xsl:when>
               <!-- With local complex type definition -->
               <xsl:when test="xsd:complexType">
                  <xsl:call-template name="PrintSampleComplexElement">
                     <xsl:with-param name="element" select="."/>
                     <xsl:with-param name="type" select="xsd:complexType"/>
                     <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     <xsl:with-param name="parentGroups" select="$parentGroups"/>
                  </xsl:call-template>
               </xsl:when>
               <xsl:otherwise>
                  <xsl:call-template name="PrintSampleSimpleElement">
                     <xsl:with-param name="element" select="."/>
                     <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                  </xsl:call-template>
               </xsl:otherwise>
            </xsl:choose>
         </xsl:when>
         <!-- Local element declaration -->
         <xsl:otherwise>
            <xsl:choose>
               <!-- With local complex type definition -->
               <xsl:when test="xsd:complexType">
                  <xsl:call-template name="PrintSampleComplexElement">
                     <xsl:with-param name="element" select="."/>
                     <xsl:with-param name="type" select="xsd:complexType"/>
                     <xsl:with-param name="margin" select="$margin"/>
                     <xsl:with-param name="isInherited" select="$isInherited"/>
                     <xsl:with-param name="isNewField" select="$isNewField"/>
                     <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     <xsl:with-param name="typeList" select="$typeList"/>
                     <xsl:with-param name="parentGroups" select="$parentGroups"/>
                  </xsl:call-template>
               </xsl:when>
               <xsl:otherwise>
                  <xsl:call-template name="PrintSampleSimpleElement">
                     <xsl:with-param name="element" select="."/>
                     <xsl:with-param name="margin" select="$margin"/>
                     <xsl:with-param name="isInherited" select="$isInherited"/>
                     <xsl:with-param name="isNewField" select="$isNewField"/>
                     <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     <xsl:with-param name="typeList" select="$typeList"/>
                  </xsl:call-template>
               </xsl:otherwise>
            </xsl:choose>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Prints out a sample XML instance from an element
     reference.
     Param(s):
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            isInherited (boolean) optional
                If true, display elements using 'inherited' CSS class.
            isNewField (boolean) optional
                If true, display elements using 'newFields' CSS class.
            schemaLoc (String) optional
                Schema file containing this element reference; 
                if in current schema, 'schemaLoc' is set to 'this'.
     -->
   <xsl:template match="xsd:element[@ref]" mode="sample">
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>

      <xsl:if test="normalize-space(@maxOccurs)!='0'">
         <xsl:call-template name="PrintSampleSimpleElement">
            <xsl:with-param name="element" select="."/>
            <xsl:with-param name="margin" select="$margin"/>
            <xsl:with-param name="isInherited" select="$isInherited"/>
            <xsl:with-param name="isNewField" select="$isNewField"/>
            <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
         </xsl:call-template>
      </xsl:if>
   </xsl:template>

   <!--
     Prints out a sample XML instance from a group definition.
     Param(s):
            schemaLoc (String) optional
                Schema file containing this model group definition;
                if in current schema, 'schemaLoc' is set to 'this'
            typeList (String) optional
                List of types in this call chain. Name of type starts
                with '*', and ends with '+'. (Used to prevent infinite
                recursive loop.)
     -->
   <xsl:template match="xsd:group[@name]" mode="sample">
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>

      <xsl:apply-templates select="xsd:*" mode="sample">
         <xsl:with-param name="parentGroups" select="concat('*', normalize-space(@name), '+')"/>
         <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
         <xsl:with-param name="typeList" select="$typeList"/>
      </xsl:apply-templates>
   </xsl:template>

   <!--
     Prints out a sample XML instance from a group reference.
     Param(s):
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            isInherited (boolean) optional
                If true, display elements using 'inherited' CSS 
                class.
            isNewField (boolean) optional
                If true, display elements using 'newFields' CSS 
                class.
            parentGroups (String) optional
                List of parent model group definitions that contain 
                this  model group. Used to prevent infinite loops 
                when displaying model group definitions.
            schemaLoc (String) optional
                Schema file containing this model group reference;
                if in current schema, 'schemaLoc' is set to 'this'
            typeList (String) optional
                List of types in this call chain. Name of type starts
                with '*', and ends with '+'. (Used to prevent infinite
                recursive loop.)
     -->
   <xsl:template match="xsd:group[@ref]" mode="sample">
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="parentGroups"/>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>

      <!-- Get group name -->
      <xsl:variable name="grpName">
         <xsl:call-template name="GetRefName">
            <xsl:with-param name="ref" select="@ref"/>
         </xsl:call-template>
      </xsl:variable>

      <!-- Create link to the group definition -->
      <xsl:variable name="grpLink">
         <xsl:call-template name="PrintGroupRef">
            <xsl:with-param name="ref" select="@ref"/>
            <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
         </xsl:call-template>
      </xsl:variable>

      <!-- Get occurrence info -->
      <xsl:variable name="occursInfo">
         <xsl:call-template name="PrintOccurs">
            <xsl:with-param name="component" select="."/>
         </xsl:call-template>
      </xsl:variable>

      <xsl:choose>
         <!-- Circular group definition -->
         <xsl:when test="contains($parentGroups, concat('*', normalize-space($grpName), '+'))">
            <!-- Don't show contents -->
            <div class="other" style="margin-left: {$margin}em">
               <xsl:text>Circular model group reference: </xsl:text>
               <xsl:copy-of select="$grpLink"/>
               <!-- Occurrence info -->
               <xsl:text> </xsl:text>
               <xsl:copy-of select="$occursInfo"/>
               <!-- Documentation -->
               <xsl:call-template name="PrintSampleDocumentation">
                  <xsl:with-param name="component" select="."/>
               </xsl:call-template>
            </div>
         </xsl:when>
         <xsl:otherwise>
            <!-- Look for group definition -->
            <xsl:variable name="grpDefLoc">
               <xsl:call-template name="FindComponent">
                  <xsl:with-param name="ref" select="@ref"/>
                  <xsl:with-param name="compType">group</xsl:with-param>
               </xsl:call-template>
            </xsl:variable>

            <xsl:choose>
               <!-- Not found -->
               <xsl:when test="normalize-space($grpDefLoc)='' or normalize-space($grpDefLoc)='none' or normalize-space($grpDefLoc)='xml' or normalize-space($grpDefLoc)='xsd'">
                  <div class="other" style="margin-left: {$margin}em">
                     <xsl:text>Model group reference (not shown): </xsl:text>
                     <xsl:copy-of select="$grpLink"/>
                     <!-- Occurrence info -->
                     <xsl:text> </xsl:text>
                     <xsl:copy-of select="$occursInfo"/>
                     <!-- Documentation -->
                     <xsl:call-template name="PrintSampleDocumentation">
                        <xsl:with-param name="component" select="."/>
                     </xsl:call-template>
                  </div>
               </xsl:when>
               <!-- Found in current schema -->
               <xsl:when test="normalize-space($grpDefLoc)='this'">
                  <xsl:variable name="grpDef" select="key('group', $grpName)"/>
                  <xsl:call-template name="PrintSampleGroup">
                     <xsl:with-param name="margin" select="$margin"/>
                     <xsl:with-param name="isInherited" select="$isInherited"/>
                     <xsl:with-param name="isNewField" select="$isNewField"/>
                     <xsl:with-param name="parentGroups" select="concat($parentGroups, concat('*', normalize-space($grpName), '+'))"/>
                     <xsl:with-param name="occursInfo" select="$occursInfo"/>
                     <xsl:with-param name="grpLink" select="$grpLink"/>
                     <xsl:with-param name="grpRef" select="."/>
                     <xsl:with-param name="grpDef" select="$grpDef"/>
                     <xsl:with-param name="grpDefLoc" select="$grpDefLoc"/>
                     <xsl:with-param name="typeList" select="$typeList"/>
                  </xsl:call-template>
               </xsl:when>
               <!-- Found in external schema -->
               <xsl:otherwise>
                  <xsl:variable name="grpDef" select="document($grpDefLoc)/xsd:schema/xsd:group[@name=$grpName]"/>
                  <xsl:call-template name="PrintSampleGroup">
                     <xsl:with-param name="margin" select="$margin"/>
                     <xsl:with-param name="isInherited" select="$isInherited"/>
                     <xsl:with-param name="isNewField" select="$isNewField"/>
                     <xsl:with-param name="parentGroups" select="concat($parentGroups, concat('*', normalize-space($grpName), '+'))"/>
                     <xsl:with-param name="occursInfo" select="$occursInfo"/>
                     <xsl:with-param name="grpLink" select="$grpLink"/>
                     <xsl:with-param name="grpRef" select="."/>
                     <xsl:with-param name="grpDef" select="$grpDef"/>
                     <xsl:with-param name="grpDefLoc" select="$grpDefLoc"/>
                     <xsl:with-param name="typeList" select="$typeList"/>
                  </xsl:call-template>
               </xsl:otherwise>
            </xsl:choose>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Prints out a sample XML instance from a 'sequence' model group.
     Param(s):
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            isInherited (boolean) optional
                If true, display elements using 'inherited' CSS class.
            isNewField (boolean) optional
                If true, display elements using 'newFields' CSS class.
            parentGroups (String) optional
                List of parent model group definitions that contain 
                this model group. Used to prevent infinite loops when 
                displaying model group definitions.
            schemaLoc (String) optional
                Schema file containing this sequence model group;
                if in current schema, 'schemaLoc' is set to 'this'
            typeList (String) optional
                List of types in this call chain. Name of type starts
                with '*', and ends with '+'. (Used to prevent infinite
                recursive loop.)
     -->
   <xsl:template match="xsd:sequence" mode="sample">
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="parentGroups"/>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>

      <xsl:if test="normalize-space(@maxOccurs)!='0'">
         <!-- Get occurrence info -->
         <xsl:variable name="occursInfo">
            <xsl:call-template name="PrintOccurs">
               <xsl:with-param name="component" select="."/>
            </xsl:call-template>
         </xsl:variable>

         <xsl:apply-templates select="xsd:*" mode="sample">
            <xsl:with-param name="margin">
               <xsl:choose>
                  <xsl:when test="normalize-space($occursInfo)='[1]'">
                     <xsl:value-of select="$margin"/>
                  </xsl:when>
                  <xsl:otherwise>
                     <xsl:value-of select="0"/>
                  </xsl:otherwise>
               </xsl:choose>
            </xsl:with-param>
            <xsl:with-param name="isInherited" select="$isInherited"/>
            <xsl:with-param name="isNewField" select="$isNewField"/>
            <xsl:with-param name="parentGroups" select="$parentGroups"/>
            <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            <xsl:with-param name="typeList" select="$typeList"/>
         </xsl:apply-templates>
      </xsl:if>
   </xsl:template>

   <!--
     Prints out the constraints of a complex type with simple content 
     to be displayed within a sample XML instance.
     Param(s):
            schemaLoc (String) optional
                Schema file containing this simple content 
                restriction; if in current schema, 'schemaLoc' is 
                set to 'this'
     -->
   <xsl:template match="xsd:simpleContent[xsd:restriction]" mode="sample">
      <xsl:param name="schemaLoc">this</xsl:param>
         <xsl:call-template name="PrintSampleSimpleRestriction">
            <xsl:with-param name="restriction" select="./xsd:restriction"/>
            <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
         </xsl:call-template>
   </xsl:template>

   <!--
     Prints out the constraints of a simple type definition to be 
     displayed within a sample XML instance.
     Param(s):
            schemaLoc (String) optional
                Schema file containing this simple type definition; 
                if in current schema, 'schemaLoc' is set to 'this'
     -->
   <xsl:template match="xsd:simpleType" mode="sample">
      <xsl:param name="schemaLoc">this</xsl:param>
         <xsl:call-template name="PrintSampleSimpleConstraints">
            <xsl:with-param name="simpleContent" select="."/>
            <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
         </xsl:call-template>
   </xsl:template>

   <!--
     Prints out the identity constraints of an element to be displayed 
     within a sample XML instance.
     Param(s):
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            schemaLoc (String) optional
                Schema file containing this simple type definition; 
                if in current schema, 'schemaLoc' is set to 'this'
     -->
   <xsl:template match="xsd:unique | xsd:key | xsd:keyref" mode="sample">
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>

      <div class="other" style="margin-left: {$margin}em">
         <xsl:text>&lt;!-- </xsl:text><br/>

         <xsl:choose>
            <xsl:when test="local-name(.)='unique'">
               <xsl:call-template name="PrintGlossaryTermRef">
                  <xsl:with-param name="code">Unique</xsl:with-param>
                  <xsl:with-param name="term">Uniqueness</xsl:with-param>
               </xsl:call-template>
            </xsl:when>
            <xsl:when test="local-name(.)='key'">
               <xsl:call-template name="PrintGlossaryTermRef">
                  <xsl:with-param name="code">Key</xsl:with-param>
                  <xsl:with-param name="term">Key</xsl:with-param>
               </xsl:call-template>
            </xsl:when>
            <xsl:otherwise>
               <xsl:call-template name="PrintGlossaryTermRef">
                  <xsl:with-param name="code">KeyRef</xsl:with-param>
                  <xsl:with-param name="term">Key Reference</xsl:with-param>
               </xsl:call-template>
            </xsl:otherwise>
         </xsl:choose>
         <xsl:text> Constraint - </xsl:text>
         <strong>
            <xsl:choose>
               <xsl:when test="local-name(.)='keyref'">
                  <xsl:value-of select="@name"/>
               </xsl:when>
               <xsl:otherwise>
                  <xsl:variable name="componentID">
                     <xsl:call-template name="GetComponentID">
                        <xsl:with-param name="component" select="."/>
                     </xsl:call-template>
                  </xsl:variable>
                  <a name="{$componentID}"><xsl:value-of select="@name"/></a>
               </xsl:otherwise>
            </xsl:choose>
         </strong><br/>

         <xsl:text>Selector - </xsl:text>
         <strong>
            <xsl:value-of select="xsd:selector/@xpath"/>
         </strong><br/>

         <xsl:text>Field(s) - </xsl:text>
         <xsl:for-each select="xsd:field">
            <xsl:if test="position()!=1">
               <xsl:text>, </xsl:text>
            </xsl:if>
            <strong>
               <xsl:value-of select="@xpath"/>
            </strong>
         </xsl:for-each>
         <br/>

         <xsl:if test="local-name(.)='keyref'">
            <xsl:text>Refers to - </xsl:text>
            <xsl:call-template name="PrintKeyRef">
               <xsl:with-param name="ref" select="@refer"/>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
            <br/>
         </xsl:if>

         <xsl:text>--></xsl:text>
      </div>
   </xsl:template>

   <!--
     Unmatched template for 'sample' mode
     -->
   <xsl:template match="*" mode="sample"/>

   <!--
     Prints out a link which will open up a window, displaying a 
     schema component's documentation.
     Param(s):
            component (Node) required
                Schema component
  -->
   <xsl:template name="PrintSampleDocumentation">
      <xsl:param name="component"/>
   </xsl:template>

   <!--
     Translates occurrences of single and double quotes
     in a piece of text with single and double quote
     escape characters.
     Param(s):
            value (String) required
                Text to translate
     -->
   <xsl:template name="EscapeQuotes">
      <xsl:param name="value"/>

      <xsl:variable name="noSingleQuotes">
         <xsl:call-template name="TranslateStr">
            <xsl:with-param name="value" select="$value"/>
            <xsl:with-param name="strToReplace">'</xsl:with-param>
            <xsl:with-param name="replacementStr">\'</xsl:with-param>
         </xsl:call-template>
      </xsl:variable>
      <xsl:variable name="noDoubleQuotes">
         <xsl:call-template name="TranslateStr">
            <xsl:with-param name="value" select="$noSingleQuotes"/>
            <xsl:with-param name="strToReplace">"</xsl:with-param>
            <xsl:with-param name="replacementStr">\"</xsl:with-param>
         </xsl:call-template>
      </xsl:variable>
      <xsl:value-of select="$noDoubleQuotes"/>
   </xsl:template>

   <!--
     Helper template for template, match="xsd:group[@ref]" 
     mode="sample". Basically prints out a group reference, for 
     which we are able to look up the group definition that it
     is referring to. This template is a work-around because XSLT
     doesn't have variables (in the traditional sense of 
     programming languages) and it doesn't allow you to query
     result tree fragments. 
     Param(s):
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            isInherited (boolean) optional
                If true, display elements using 'inherited' CSS class.
            isNewField (boolean) optional
                If true, display elements using 'newFields' CSS class.
            parentGroups (String) optional
                List of parent model group definitions that contain this 
                model group. Used to prevent infinite loops when 
                displaying model group definitions.
            occursInfo (Result tree fragment) required
                Pre-formatted occurrence info of group reference
            grpLink (Result tree fragment) required
                Pre-formatted <a> link representing group reference
            grpRef (Node) required
                Group reference
            grpDef (Node) required
                Group definition that the reference is pointing to
            grpDefLoc (String) optional
                Schema file containing 'grpDef' group definition;
                if current schema, 'schemaLoc' is set to 'this'
            typeList (String) optional
                List of types in this call chain. Name of type starts
                with '*', and ends with '+'. (Used to prevent infinite
                recursive loop.)
     -->
   <xsl:template name="PrintSampleGroup">
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="parentGroups"/>
      <xsl:param name="occursInfo"/>
      <xsl:param name="grpLink"/>
      <xsl:param name="grpRef"/>
      <xsl:param name="grpDef"/>
      <xsl:param name="grpDefLoc">this</xsl:param>
      <xsl:param name="typeList"/>

      <!-- Header -->
      <xsl:if test="normalize-space($occursInfo)!='[1]'">
         <!-- Don't print out header if min/max occurs is one. -->
            <xsl:text>Start Group: </xsl:text>
            <xsl:copy-of select="$grpLink"/>
            <!-- Occurrence info -->
            <xsl:text> </xsl:text>
            <xsl:copy-of select="$occursInfo"/>
            <!-- Documentation -->
            <xsl:call-template name="PrintSampleDocumentation">
               <xsl:with-param name="component" select="$grpRef"/>
            </xsl:call-template>
      </xsl:if>

      <!-- Content -->
      <xsl:apply-templates select="$grpDef/xsd:*" mode="sample">
         <xsl:with-param name="margin">
            <xsl:choose>
               <xsl:when test="normalize-space($occursInfo)!='[1]'">
                  <xsl:value-of select="0"/>
               </xsl:when>
               <xsl:otherwise>
                  <xsl:value-of select="$margin"/>
               </xsl:otherwise>
            </xsl:choose>
         </xsl:with-param>
         <xsl:with-param name="isInherited" select="$isInherited"/>
         <xsl:with-param name="isNewField" select="$isNewField"/>
         <xsl:with-param name="parentGroups" select="$parentGroups"/>
         <xsl:with-param name="schemaLoc" select="$grpDefLoc"/>
         <xsl:with-param name="typeList" select="$typeList"/>
      </xsl:apply-templates>

      <!-- Footer -->
      <xsl:if test="normalize-space($occursInfo)!='[1]'">
         <!-- Don't print out footer if min/max occurs is one. -->
            <xsl:text>End Group: </xsl:text>
            <xsl:copy-of select="$grpLink"/>
      </xsl:if>
   </xsl:template>

   <!--
     Prints out a sample element instance in one line.
     Param(s):
            element (Node) required
                Element declaration or reference
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            isInherited (boolean) optional
                If true, display element using 'inherited' CSS class.
            isNewField (boolean) optional
                If true, display element using 'newFields' CSS class.
            schemaLoc (String) optional
                Schema file containing this element declaration
                or reference; if in current schema, 'schemaLoc' is 
                set to 'this'.
     -->
   <xsl:template name="PrintSampleSimpleElement">
      <xsl:param name="element"/>
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>

      <!-- Element Tag -->
      <xsl:variable name="elemTag">
         <!-- Local Name -->
         <xsl:choose>
            <!-- Element reference -->
            <xsl:when test="$element/@ref">
               <!-- Note: Prefix will be automatically written out
                    in call to 'PrintElementRef'. -->
               <xsl:call-template name="PrintElementRef">
                  <xsl:with-param name="ref" select="@ref"/>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:call-template>
            </xsl:when>
            <!-- Element declaration -->
            <xsl:otherwise>
               <!-- Prefix -->
               <xsl:variable name="prefix">
                  <xsl:call-template name="GetElementPrefix">
                     <xsl:with-param name="element" select="$element"/>
                  </xsl:call-template>
               </xsl:variable>
               <xsl:value-of select="$element/@name"/>
            </xsl:otherwise>
         </xsl:choose>
      </xsl:variable>

         <xsl:choose>
            <xsl:when test="$isNewField!='false'">
               <xsl:attribute name="class">newFields</xsl:attribute>
            </xsl:when>
            <xsl:when test="$isInherited!='false'">
               <xsl:attribute name="class">inherited</xsl:attribute>
            </xsl:when>
         </xsl:choose>

         <!-- Start Tag -->
     <xsl:text>
       </xsl:text>
         <xsl:text>&lt;</xsl:text>
         <xsl:copy-of select="$elemTag"/>
         <xsl:text>></xsl:text>

         <!-- Contents -->
         <xsl:text> </xsl:text>
         <xsl:choose>
            <!-- Fixed value is provided -->
            <xsl:when test="$element/@fixed">
                  <xsl:value-of select="$element/@fixed"/>
            </xsl:when>
            <!-- Type reference is provided -->
            <xsl:when test="$element/@name and $element/@type">
               <xsl:call-template name="PrintTypeRef">
                  <xsl:with-param name="ref" select="$element/@type"/>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:call-template>
            </xsl:when>
            <!-- Local simple type definition is provided -->
            <xsl:when test="$element/@name and $element/xsd:simpleType">
               <xsl:apply-templates select="$element/xsd:simpleType" mode="sample">
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:apply-templates>
            </xsl:when>
            <xsl:otherwise>
               <xsl:text>...</xsl:text>
            </xsl:otherwise>
         </xsl:choose>
         <xsl:text> </xsl:text>

         <!-- Identity Constraints -->
         <xsl:if test="$element/xsd:unique or $element/xsd:key or $element/xsd:keyref">
            <xsl:apply-templates select="$element/xsd:unique | $element/xsd:key | $element/xsd:keyref" mode="sample">
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:apply-templates>
         </xsl:if>

         <!-- End Tag -->
         <xsl:text>&lt;/</xsl:text>
         <xsl:copy-of select="$elemTag"/>
         <xsl:text>></xsl:text>

         <xsl:if test="local-name($element/..)!='schema'">
            <!-- Min/max occurs information -->
            <xsl:text> </xsl:text>
            <xsl:call-template name="PrintOccurs">
               <xsl:with-param name="component" select="$element"/>
            </xsl:call-template>
            <!-- Documentation -->
            <xsl:call-template name="PrintSampleDocumentation">
               <xsl:with-param name="component" select="$element"/>
            </xsl:call-template>
         </xsl:if>
   </xsl:template>

   <!--
     Prints out a sample element instance that has complex content.
     Param(s):
            type (Node) required
                Complex type definition
            element (Node) optional
                Element declaration
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            isInherited (boolean) optional
                If true, display element using 'inherited' CSS class.
            isNewField (boolean) optional
                If true, display element using 'newFields' CSS class.
            schemaLoc (String) optional
                Schema file containing this element declaration
                or type definition; if in current schema, 'schemaLoc' 
                is set to 'this'.
            typeList (String) optional
                List of types in this call chain. Name of type starts
                with '*', and ends with '+'. (Used to prevent infinite
                recursive loop.)
     -->
   <xsl:template name="PrintSampleComplexElement">
      <xsl:param name="type"/>
      <xsl:param name="element"/>
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>
      <xsl:param name="parentGroups"/>

      <xsl:choose>
         <!-- Circular type hierarchy -->
         <xsl:when test="$type/@name and contains($typeList, concat('*', $type/@name, '+'))"/>
         <xsl:otherwise>
            <xsl:variable name="tag">
               <xsl:choose>
                  <xsl:when test="$element">
                     <!-- Prefix -->
                     <xsl:variable name="prefix">
                        <xsl:call-template name="GetElementPrefix">
                           <xsl:with-param name="element" select="$element"/>
                        </xsl:call-template>
                     </xsl:variable>
                     <xsl:value-of select="$element/@name"/>
                  </xsl:when>
                  <xsl:otherwise>
                     <xsl:text>...</xsl:text>
                  </xsl:otherwise>
               </xsl:choose>
            </xsl:variable>

            <xsl:variable name="fromTopCType">
               <xsl:choose>
                  <xsl:when test="not($element) and local-name($type/..)='schema'">
                     <xsl:text>true</xsl:text>
                  </xsl:when>
                  <xsl:otherwise>
                     <xsl:text>false</xsl:text>
                  </xsl:otherwise>
               </xsl:choose>
            </xsl:variable>

           <xsl:choose>
                  <xsl:when test="$isNewField!='false'">
                     <xsl:attribute name="class">newFields</xsl:attribute>
                  </xsl:when>
                  <xsl:when test="$isInherited!='false'">
                     <xsl:attribute name="class">inherited</xsl:attribute>
                  </xsl:when>
               </xsl:choose>
         
               <!-- Start Tag -->
               <xsl:text>&lt;</xsl:text>
               <xsl:copy-of select="$tag"/>

               <!-- Print attributes -->
               <xsl:call-template name="PrintSampleTypeAttrs">
                  <xsl:with-param name="type" select="$type"/>
                  <xsl:with-param name="isInherited" select="$isInherited"/>
                  <xsl:with-param name="isNewField" select="$isNewField"/>
                  <xsl:with-param name="fromTopCType" select="$fromTopCType"/>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                  <xsl:with-param name="typeList" select="$typeList"/>
               </xsl:call-template>

               <!-- Get content -->
               <xsl:variable name="content">
                  <xsl:call-template name="PrintSampleTypeContent">
                     <xsl:with-param name="type" select="$type"/>
                     <xsl:with-param name="isInherited" select="$isInherited"/>
                     <xsl:with-param name="isNewField" select="$isNewField"/>
                     <xsl:with-param name="fromTopCType" select="$fromTopCType"/>
                     <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     <xsl:with-param name="typeList" select="$typeList"/>
                     <xsl:with-param name="parentGroups" select="$parentGroups"/>
                  </xsl:call-template>
               </xsl:variable>

               <!-- Find out if content type is mixed -->
               <xsl:variable name="mixed">
                  <xsl:choose>
                     <xsl:when test="normalize-space(translate($type/xsd:complexContent/@mixed, 'TRUE', 'true'))='true' or normalize-space($type/xsd:complexContent/@mixed)='1'">
                        <xsl:text>true</xsl:text>
                     </xsl:when>
                     <xsl:when test="normalize-space(translate($type/@mixed, 'TRUE', 'true'))='true' or normalize-space($type/@mixed)='1'">
                        <xsl:text>true</xsl:text>
                     </xsl:when>
                     <xsl:otherwise>
                        <xsl:text>false</xsl:text>
                     </xsl:otherwise>
                  </xsl:choose>
               </xsl:variable>

               <!-- Find out if there are identity constraints -->
               <xsl:variable name="hasIdConstraints">
                  <xsl:if test="$element and ($element/xsd:unique or $element/xsd:key or $element/xsd:keyref)">
                     <xsl:text>true</xsl:text>
                  </xsl:if>
               </xsl:variable>

               <!-- Print content -->
               <xsl:choose>
                  <!-- Empty content -->
                  <xsl:when test="$hasIdConstraints!='true' and normalize-space($content)=''">
                     <!-- Close start tag -->
                     <xsl:text>/> </xsl:text>

                     <xsl:if test="$element and local-name($element/..)!='schema'">
                        <!-- Occurrence info -->
                        <xsl:text> </xsl:text>
                        <xsl:call-template name="PrintOccurs">
                           <xsl:with-param name="component" select="$element"/>
                        </xsl:call-template>

                        <!-- Documentation -->
                        <xsl:call-template name="PrintSampleDocumentation">
                           <xsl:with-param name="component" select="$element"/>
                        </xsl:call-template>
                     </xsl:if>
                  </xsl:when>
                  <xsl:otherwise>
                     <!-- Close start tag -->
                     <xsl:text>> </xsl:text>

                     <xsl:if test="$element and local-name($element/..)!='schema'">
                        <!-- Occurrence info -->
                        <xsl:text> </xsl:text>
                        <xsl:call-template name="PrintOccurs">
                           <xsl:with-param name="component" select="$element"/>
                        </xsl:call-template>

                        <!-- Documentation -->
                        <xsl:text> </xsl:text>
                        <xsl:call-template name="PrintSampleDocumentation">
                           <xsl:with-param name="component" select="$element"/>
                        </xsl:call-template>
                     </xsl:if>

                     <!-- Identity Constraints -->
                     <xsl:if test="$element">
                        <xsl:apply-templates select="$element/xsd:unique | $element/xsd:key | $element/xsd:keyref" mode="sample">
                        </xsl:apply-templates>
                     </xsl:if>

                     <!-- Print out restriction/extension information -->
                     <xsl:choose>
                        <xsl:when test="false()">
                        <!--<xsl:when test="$type/xsd:complexContent/xsd:restriction/@base">-->
                              <xsl:text>&lt;!-- Restricts : </xsl:text>
                              <xsl:call-template name="PrintTypeRef">
                                 <xsl:with-param name="ref" select="$type/xsd:complexContent/xsd:restriction/@base"/>
                                 <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                              </xsl:call-template>
                              <xsl:text> --></xsl:text>
                        </xsl:when>
                        <xsl:when test="false()">
                              <xsl:text>&lt;!-- Extends : </xsl:text>
                              <xsl:call-template name="PrintTypeRef">
                                 <xsl:with-param name="ref" select="$type/xsd:complexContent/xsd:extension/@base"/>
                                 <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                              </xsl:call-template>
                              <xsl:text> --></xsl:text>
                        </xsl:when>
                     </xsl:choose>

                     <!-- Print out message if has mixed content -->
                     <xsl:if test="$mixed='true'">
                           <xsl:text>&lt;!-- Mixed content --></xsl:text>
                     </xsl:if>

                     <!-- Element Content -->
                     <xsl:copy-of select="$content"/>

                     <!-- End Tag -->
                    <xsl:text>
    &lt;/</xsl:text>
                     <xsl:copy-of select="$tag"/>
                     <xsl:text>></xsl:text>
                  </xsl:otherwise>
               </xsl:choose>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Prints out attributes of a complex type definition, including 
     those inherited from a base type.
     Param(s):
            type (Node) required
                Complex type definition
            subTypeAttrs (String) optional
                List of attributes in sub-types of this current type 
                definition
            isInherited (boolean) optional
                If true, display attributes using 'inherited' CSS class.
            isNewField (boolean) optional
                If true, display attributes using 'newFields' CSS class.
            fromTopCType (boolean) optional
                Set to true if this is being displayed in the XML 
                Instance Representation table of a top-level complex 
                type definition, in which case, 'inherited' attributes 
                and elements are distinguished.
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            schemaLoc (String) optional
                Schema file containing this complex type definition; 
                if in current schema, 'schemaLoc' is set to 'this'.
            typeList (String) optional
                List of types in this call chain. Name of type starts
                with '*', and ends with '+'. (Used to prevent infinite
                recursive loop.)
     -->
   <xsl:template name="PrintSampleTypeAttrs">
      <xsl:param name="type"/>
      <xsl:param name="subTypeAttrs"/>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="fromTopCType">false</xsl:param>
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>

      <xsl:choose>
         <!-- Circular type hierarchy -->
         <xsl:when test="$type/@name and contains($typeList, concat('*', $type/@name, '+'))">
            <!-- Do nothing. 
                 Error message will be written out by 'PrintSampleTypeContent' template. 
            -->
         </xsl:when>
         <!-- Derivation -->
         <xsl:when test="$type/xsd:complexContent or $type/xsd:simpleContent">
            <xsl:choose>
               <xsl:when test="$type/xsd:complexContent/xsd:restriction">
                  <xsl:call-template name="PrintSampleDerivedTypeAttrs">
                     <xsl:with-param name="derivationElem" select="$type/xsd:complexContent/xsd:restriction"/>
                     <xsl:with-param name="subTypeAttrs" select="$subTypeAttrs"/>
                     <xsl:with-param name="isInherited" select="$isInherited"/>
                     <xsl:with-param name="isNewField" select="$isNewField"/>
                     <xsl:with-param name="fromTopCType" select="$fromTopCType"/>
                     <xsl:with-param name="margin" select="$margin"/>
                     <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     <xsl:with-param name="typeList" select="concat($typeList, '*', $type/@name, '+')"/>
                  </xsl:call-template>
               </xsl:when>
               <xsl:when test="$type/xsd:simpleContent/xsd:restriction">
                  <xsl:call-template name="PrintSampleDerivedTypeAttrs">
                     <xsl:with-param name="derivationElem" select="$type/xsd:simpleContent/xsd:restriction"/>
                     <xsl:with-param name="subTypeAttrs" select="$subTypeAttrs"/>
                     <xsl:with-param name="isInherited" select="$isInherited"/>
                     <xsl:with-param name="isNewField" select="$isNewField"/>
                     <xsl:with-param name="fromTopCType" select="$fromTopCType"/>
                     <xsl:with-param name="margin" select="$margin"/>
                     <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     <xsl:with-param name="typeList" select="concat($typeList, '*', $type/@name, '+')"/>
                  </xsl:call-template>
               </xsl:when>
               <xsl:when test="$type/xsd:complexContent/xsd:extension">
                  <xsl:call-template name="PrintSampleDerivedTypeAttrs">
                     <xsl:with-param name="derivationElem" select="$type/xsd:complexContent/xsd:extension"/>
                     <xsl:with-param name="subTypeAttrs" select="$subTypeAttrs"/>
                     <xsl:with-param name="isInherited" select="$isInherited"/>
                     <xsl:with-param name="isNewField" select="$isNewField"/>
                     <xsl:with-param name="fromTopCType" select="$fromTopCType"/>
                     <xsl:with-param name="margin" select="$margin"/>
                     <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     <xsl:with-param name="typeList" select="concat($typeList, '*', $type/@name, '+')"/>
                  </xsl:call-template>
               </xsl:when>
               <xsl:when test="$type/xsd:simpleContent/xsd:extension">
                  <xsl:call-template name="PrintSampleDerivedTypeAttrs">
                     <xsl:with-param name="derivationElem" select="$type/xsd:simpleContent/xsd:extension"/>
                     <xsl:with-param name="subTypeAttrs" select="$subTypeAttrs"/>
                     <xsl:with-param name="isInherited" select="$isInherited"/>
                     <xsl:with-param name="isNewField" select="$isNewField"/>
                     <xsl:with-param name="fromTopCType" select="$fromTopCType"/>
                     <xsl:with-param name="margin" select="$margin"/>
                     <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     <xsl:with-param name="typeList" select="concat($typeList, '*', $type/@name, '+')"/>
                  </xsl:call-template>
               </xsl:when>
            </xsl:choose>
         </xsl:when>
         <!-- No derivation -->
         <xsl:when test="local-name($type)='complexType'">
            <xsl:call-template name="PrintSampleAttrList">
               <xsl:with-param name="list" select="$type"/>
               <xsl:with-param name="subTypeAttrs" select="$subTypeAttrs"/>
               <xsl:with-param name="isInherited" select="$isInherited"/>
               <xsl:with-param name="isNewField" select="$isNewField"/>
               <xsl:with-param name="margin" select="$margin"/>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               <xsl:with-param name="typeList" select="concat($typeList, '*', $type/@name, '+')"/>
            </xsl:call-template>
         </xsl:when>
         <!-- Ignore base types that are simple types -->
      </xsl:choose>
   </xsl:template>

   <!--
     Helper function 'PrintSampleTypeAttrs' template to
     handle case of derived types.
     Param(s):
            derivationElem (Node) required
                'restriction' or 'extension' element
            subTypeAttrs (String) optional
                List of attributes in sub-types of 
                this current type definition
            isInherited (boolean) optional
                If true, display attributes using 'inherited' CSS class.
            isNewField (boolean) optional
                If true, display attributes using 'newFields' CSS class.
            fromTopCType (boolean) optional
                Set to true if this is being displayed 
                in the XML Instance Representation table 
                of a top-level complex type definition, in
                which case, 'inherited' attributes and
                elements are distinguished.
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            schemaLoc (String) optional
                Schema file containing this derivation element; 
                if in current schema, 'schemaLoc' is set to 'this'.
            typeList (String) optional
                List of types in this call chain. Name of type starts
                with '*', and ends with '+'. (Used to prevent infinite
                recursive loop.)
     -->
   <xsl:template name="PrintSampleDerivedTypeAttrs">
      <xsl:param name="derivationElem"/>
      <xsl:param name="subTypeAttrs"/>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="fromTopCType">false</xsl:param>
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>

      <!-- Get attributes from this type to add to 
            'subTypeAttrs' list for recursive call on base type -->
      <xsl:variable name="thisAttrs">
         <xsl:call-template name="GetAttrList">
            <xsl:with-param name="list" select="$derivationElem"/>
         </xsl:call-template>
      </xsl:variable>

      <!-- Look for base type -->
      <xsl:variable name="baseTypeName">
         <xsl:call-template name="GetRefName">
            <xsl:with-param name="ref" select="$derivationElem/@base"/>
         </xsl:call-template>
      </xsl:variable>
      <xsl:variable name="defLoc">
         <xsl:call-template name="FindComponent">
            <xsl:with-param name="ref" select="$derivationElem/@base"/>
            <xsl:with-param name="compType">complex type</xsl:with-param>
         </xsl:call-template>
      </xsl:variable>
      <xsl:choose>
         <!-- Complex type was found in current schema. -->
         <xsl:when test="normalize-space($defLoc)='this'">
            <xsl:variable name="ctype" select="key('complexType', $baseTypeName)"/>
            <xsl:call-template name="PrintSampleTypeAttrs">
               <xsl:with-param name="type" select="$ctype"/>
               <xsl:with-param name="subTypeAttrs" select="concat($subTypeAttrs, $thisAttrs)"/>
               <xsl:with-param name="isInherited">
                  <xsl:choose>
                     <xsl:when test="$fromTopCType!='false'">
                        <xsl:text>true</xsl:text>
                     </xsl:when>
                     <xsl:otherwise>
                        <xsl:text>false</xsl:text>
                     </xsl:otherwise>
                  </xsl:choose>
               </xsl:with-param>
               <xsl:with-param name="isNewField" select="$isNewField"/>
               <xsl:with-param name="fromTopCType" select="$fromTopCType"/>
               <xsl:with-param name="margin" select="$margin"/>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               <xsl:with-param name="typeList" select="$typeList"/>
            </xsl:call-template>
         </xsl:when>
         <!-- Complex type was not found. -->
         <xsl:when test="normalize-space($defLoc)='' or normalize-space($defLoc)='none' or normalize-space($defLoc)='xml' or normalize-space($defLoc)='xsd'">
            <!-- IGNORE: Error message will be printed out be
                 'PrintSampleTypeContent' template. -->
         </xsl:when>
         <!-- Complex type was found in external schema. -->
         <xsl:otherwise>
            <xsl:variable name="ctype" select="document($defLoc)/xsd:schema/xsd:complexType[@name=$baseTypeName]"/>
            <xsl:call-template name="PrintSampleTypeAttrs">
               <xsl:with-param name="type" select="$ctype"/>
               <xsl:with-param name="subTypeAttrs" select="concat($subTypeAttrs, $thisAttrs)"/>
               <xsl:with-param name="isInherited">
                  <xsl:choose>
                     <xsl:when test="$fromTopCType!='false'">
                        <xsl:text>true</xsl:text>
                     </xsl:when>
                     <xsl:otherwise>
                        <xsl:text>false</xsl:text>
                     </xsl:otherwise>
                  </xsl:choose>
               </xsl:with-param>
               <xsl:with-param name="isNewField" select="$isNewField"/>
               <xsl:with-param name="fromTopCType" select="$fromTopCType"/>
               <xsl:with-param name="margin" select="$margin"/>
               <xsl:with-param name="schemaLoc" select="$defLoc"/>
               <xsl:with-param name="typeList" select="$typeList"/>
            </xsl:call-template>
         </xsl:otherwise>
      </xsl:choose>

      <!-- Print out attributes in this type -->
      <xsl:call-template name="PrintSampleAttrList">
         <xsl:with-param name="list" select="$derivationElem"/>
         <xsl:with-param name="subTypeAttrs" select="$subTypeAttrs"/>
         <xsl:with-param name="isInherited" select="$isInherited"/>
         <xsl:with-param name="isNewField">
            <xsl:choose>
               <xsl:when test="$fromTopCType!='false' and $isInherited='false'">
                  <xsl:text>true</xsl:text>
               </xsl:when>
               <xsl:otherwise>
                  <xsl:text>false</xsl:text>
               </xsl:otherwise>
            </xsl:choose>
         </xsl:with-param>
         <xsl:with-param name="margin" select="$margin"/>
         <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
      </xsl:call-template>
   </xsl:template>

   <!--
     Returns the names and namespaces of attributes
     in a list of attributes and attribute groups.
     Param(s):
            list (Node) required
                Node containing list of attributes and attribute groups
     -->
   <xsl:template name="GetAttrList">
      <xsl:param name="list"/>

      <xsl:if test="$list">
         <xsl:for-each select="$list/xsd:attribute | $list/xsd:attributeGroup | $list/xsd:anyAttribute">
            <xsl:choose>
               <!-- Attribute declaration -->
               <xsl:when test="local-name(.)='attribute' and @name">
                  <!-- Get attribute name -->
                  <xsl:variable name="attrName" select="@name"/>
                  <!-- Get attribute namespace -->
                  <xsl:variable name="attrNS">
                     <xsl:call-template name="GetAttributeNS">
                        <xsl:with-param name="attribute" select="."/>
                     </xsl:call-template>
                  </xsl:variable>

                  <xsl:value-of select="concat('*', normalize-space($attrNS), '+', normalize-space($attrName), '+')"/>
               </xsl:when>
               <!-- Attribute reference -->
               <xsl:when test="local-name(.)='attribute' and @ref">
                  <!-- Get attribute name -->
                  <xsl:variable name="attrName">
                     <xsl:call-template name="GetRefName">
                        <xsl:with-param name="ref" select="@ref"/>
                     </xsl:call-template>
                  </xsl:variable>
                  <!-- Get attribute namespace -->
                  <xsl:variable name="attrNS">
                     <xsl:call-template name="GetAttributeNS">
                        <xsl:with-param name="attribute" select="."/>
                     </xsl:call-template>
                  </xsl:variable>

                  <xsl:value-of select="concat('*', normalize-space($attrNS), '+', normalize-space($attrName), '+')"/>
               </xsl:when>
               <!-- Attribute Group reference -->
               <xsl:when test="local-name(.)='attributeGroup' and @ref">
                  <xsl:variable name="attrGrpName">
                     <xsl:call-template name="GetRefName">
                        <xsl:with-param name="ref" select="@ref"/>
                     </xsl:call-template>
                  </xsl:variable>
                  <xsl:call-template name="GetAttrList">
                     <xsl:with-param name="list" select="key('attributeGroup', $attrGrpName)"/>
                  </xsl:call-template>
               </xsl:when>
               <!-- Attribute wildcard -->
               <xsl:when test="local-name(.)='anyAttribute'">
               </xsl:when>
            </xsl:choose>
         </xsl:for-each>
      </xsl:if>
   </xsl:template>

   <!--
     Prints out sample XML instances from a list of attributes and 
     attribute groups.
     Param(s):
            list (Node) required
                Node containing list of attributes and attribute groups
            subTypeAttrs (String) optional
                List of attributes in sub-types of 
                the type definition containing this list
            isInherited (boolean) optional
                If true, display attributes using 'inherited' CSS class.
            isNewField (boolean) optional
                If true, display attributes using 'newFields' CSS class.
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            schemaLoc (String) optional
                Schema file containing this attribute list; 
                if in current schema, 'schemaLoc' is set to 'this'.
     -->
   <xsl:template name="PrintSampleAttrList">
      <xsl:param name="list"/>
      <xsl:param name="subTypeAttrs"/>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>

      <xsl:apply-templates select="$list/xsd:attribute | $list/xsd:attributeGroup | $list/xsd:anyAttribute" mode="sample">
         <xsl:with-param name="subTypeAttrs" select="$subTypeAttrs"/>
         <xsl:with-param name="isInherited" select="$isInherited"/>
         <xsl:with-param name="isNewField" select="$isNewField"/>
         <xsl:with-param name="margin" select="$margin"/>
         <xsl:with-param name="addBR">true</xsl:with-param>
         <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
      </xsl:apply-templates>
   </xsl:template>

   <!--
     Prints out the element content of a complex type definition,
     including those inherited from a base type.
     Param(s):
            type (Node) required
                Complex type definition
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            isInherited (boolean) optional
                If true, display elements using 'inherited' CSS class.
            isNewField (boolean) optional
                If true, display elements using 'newFields' CSS class.
            fromTopCType (boolean) optional
                Set to true if this is being displayed in the XML 
                Instance Representation table of a top-level complex 
                type definition, in which case, 'inherited' attributes 
                and elements are distinguished.
            addBR (boolean) optional
                If true, can add <br/> before element content.
                Applicable only if displaying complex content.
            schemaLoc (String) optional
                Schema file containing this type definition; 
                if in current schema, 'schemaLoc' is set to 'this'.
            typeList (String) optional
                List of types in this call chain. Name of type starts
                with '*', and ends with '+'. (Used to prevent infinite
                recursive loop.)
     -->
   <xsl:template name="PrintSampleTypeContent">
      <xsl:param name="type"/>
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="fromTopCType">false</xsl:param>
      <xsl:param name="addBR">true</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>
      <xsl:param name="parentGroups"/>

      <xsl:if test="$addBR='true'"><br/></xsl:if>

      <xsl:choose>
         <!-- Circular type hierarchy -->
         <xsl:when test="$type/@name and contains($typeList, concat('*', $type/@name, '+'))"/>
         <!-- Derivation by restriction on complex content -->
         <xsl:when test="$type/xsd:complexContent/xsd:restriction">
            <xsl:variable name="restriction" select="$type/xsd:complexContent/xsd:restriction"/>

            <!-- Test if base type is in schema to print out warning comment-->
            <xsl:variable name="baseTypeName">
               <xsl:call-template name="GetRefName">
                  <xsl:with-param name="ref" select="$restriction/@base"/>
               </xsl:call-template>
            </xsl:variable>
            <!-- Look for base type -->
            <xsl:variable name="defLoc">
               <xsl:call-template name="FindComponent">
                  <xsl:with-param name="ref" select="$restriction/@base"/>
                  <xsl:with-param name="compType">complex type</xsl:with-param>
               </xsl:call-template>
            </xsl:variable>

            <xsl:choose>
               <!-- Complex type was not found. -->
               <xsl:when test="normalize-space($defLoc)='' or normalize-space($defLoc)='none' or normalize-space($defLoc)='xml' or normalize-space($defLoc)='xsd'">
                  <div class="other" style="margin-left: {$margin}em;">
                     <xsl:text>&lt;!-- '</xsl:text>
                     <xsl:call-template name="PrintTypeRef">
                        <xsl:with-param name="ref" select="$restriction/@base"/>
                        <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     </xsl:call-template>
                     <xsl:text>' super type was not found in this schema. Some elements and attributes may be missing. --></xsl:text>
                  </div>
               </xsl:when>
               <!-- Complex type was found. -->
               <xsl:otherwise>
                  <!-- IGNORE element content of base type if by restriction,
                       since current content will override restricted
                       base type's content. -->
               </xsl:otherwise>
            </xsl:choose>

            <!-- Print out content from this type -->
            <xsl:if test="$restriction/xsd:*[local-name(.)!='annotation']">
               <xsl:call-template name="PrintSampleParticleList">
                  <xsl:with-param name="list" select="$restriction"/>
                  <xsl:with-param name="margin" select="$margin"/>
                  <xsl:with-param name="isInherited" select="$isInherited"/>
                  <xsl:with-param name="isNewField">
                     <xsl:choose>
                        <xsl:when test="$fromTopCType!='false' and $isInherited='false'">
                           <xsl:text>true</xsl:text>
                        </xsl:when>
                        <xsl:otherwise>
                           <xsl:text>false</xsl:text>
                        </xsl:otherwise>
                     </xsl:choose>
                  </xsl:with-param>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                  <xsl:with-param name="typeList" select="concat($typeList, '*', $type/@name, '+')"/>
                  <xsl:with-param name="parentGroups" select="$parentGroups"/>
               </xsl:call-template>
            </xsl:if>
         </xsl:when>
         <!-- Derivation by extension on complex content -->
         <xsl:when test="$type/xsd:complexContent/xsd:extension">
            <xsl:variable name="extension" select="$type/xsd:complexContent/xsd:extension"/>

            <xsl:variable name="baseTypeName">
               <xsl:call-template name="GetRefName">
                  <xsl:with-param name="ref" select="$extension/@base"/>
               </xsl:call-template>
            </xsl:variable>

            <xsl:choose>
               <xsl:when test="contains($typeList, concat('*', $baseTypeName, '+'))">
                  <div class="other" style="margin-left: {$margin}em">
                     <xsl:text>&lt;-- Extends: </xsl:text>
                     <xsl:call-template name="PrintTypeRef">
                        <xsl:with-param name="ref" select="$extension/@base"/>
                        <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                     </xsl:call-template>
                     <xsl:text> (Circular type hierarchy) --&gt;</xsl:text>
                  </div>
               </xsl:when>
               <xsl:otherwise>
                  <!-- Look for base type -->
                  <xsl:variable name="defLoc">
                     <xsl:call-template name="FindComponent">
                        <xsl:with-param name="ref" select="$extension/@base"/>
                        <xsl:with-param name="compType">complex type</xsl:with-param>
                     </xsl:call-template>
                  </xsl:variable>

                  <xsl:choose>
                     <!-- Complex type was found in current schema. -->
                     <xsl:when test="normalize-space($defLoc)='this'">
                        <xsl:variable name="ctype" select="key('complexType', $baseTypeName)"/>
                        <xsl:call-template name="PrintSampleTypeContent">
                           <xsl:with-param name="type" select="$ctype"/>
                           <xsl:with-param name="margin" select="$margin"/>
                           <xsl:with-param name="isInherited">
                              <xsl:choose>
                                 <xsl:when test="$fromTopCType!='false'">
                                    <xsl:text>true</xsl:text>
                                 </xsl:when>
                                 <xsl:otherwise>
                                    <xsl:text>false</xsl:text>
                                 </xsl:otherwise>
                              </xsl:choose>
                           </xsl:with-param>
                           <xsl:with-param name="isNewField" select="$isNewField"/>
                           <xsl:with-param name="fromTopCType" select="$fromTopCType"/>
                           <xsl:with-param name="addBR">false</xsl:with-param>
                           <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                           <xsl:with-param name="typeList" select="concat($typeList, '*', $type/@name, '+')"/>
			   <xsl:with-param name="parentGroups" select="$parentGroups"/>
                        </xsl:call-template>
                     </xsl:when>
                     <!-- Complex type was not found. -->
                     <xsl:when test="normalize-space($defLoc)='' or normalize-space($defLoc)='none' or normalize-space($defLoc)='xml' or normalize-space($defLoc)='xsd'">
                        <div class="other" style="margin-left: {$margin}em;">
                           <xsl:text>&lt;!-- '</xsl:text>
                           <xsl:call-template name="PrintTypeRef">
                              <xsl:with-param name="ref" select="$extension/@base"/>
                              <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                           </xsl:call-template>
                           <xsl:text>' super type was not found in this schema. Some elements and attributes may be missing. --></xsl:text>
                        </div>
                     </xsl:when>
                     <!-- Complex type was found in external schema. -->
                     <xsl:otherwise>
                        <xsl:variable name="ctype" select="document($defLoc)/xsd:schema/xsd:complexType[@name=$baseTypeName]"/>
                        <xsl:call-template name="PrintSampleTypeContent">
                           <xsl:with-param name="type" select="$ctype"/>
                           <xsl:with-param name="margin" select="$margin"/>
                           <xsl:with-param name="isInherited">
                              <xsl:choose>
                                 <xsl:when test="$fromTopCType!='false'">
                                    <xsl:text>true</xsl:text>
                                 </xsl:when>
                                 <xsl:otherwise>
                                    <xsl:text>false</xsl:text>
                                 </xsl:otherwise>
                              </xsl:choose>
                           </xsl:with-param>
                           <xsl:with-param name="isNewField" select="$isNewField"/>
                           <xsl:with-param name="fromTopCType" select="$fromTopCType"/>
                           <xsl:with-param name="addBR">false</xsl:with-param>
                           <xsl:with-param name="schemaLoc" select="$defLoc"/>
                           <xsl:with-param name="typeList" select="concat($typeList, '*', $type/@name, '+')"/>
			   <xsl:with-param name="parentGroups" select="$parentGroups"/>
                        </xsl:call-template>
                     </xsl:otherwise>
                  </xsl:choose>

                  <!-- Print out content from this type -->
                  <xsl:if test="$extension/xsd:*[local-name(.)!='annotation']">
                     <xsl:call-template name="PrintSampleParticleList">
                        <xsl:with-param name="list" select="$extension"/>
                        <xsl:with-param name="margin" select="$margin"/>
                        <xsl:with-param name="isInherited" select="$isInherited"/>
                        <xsl:with-param name="isNewField">
                           <xsl:choose>
                              <xsl:when test="$fromTopCType!='false' and $isInherited='false'">
                                 <xsl:text>true</xsl:text>
                              </xsl:when>
                              <xsl:otherwise>
                                 <xsl:text>false</xsl:text>
                              </xsl:otherwise>
                           </xsl:choose>
                        </xsl:with-param>
                        <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                        <xsl:with-param name="typeList" select="concat($typeList, '*', $type/@name, '+')"/>
			<xsl:with-param name="parentGroups" select="$parentGroups"/>
                     </xsl:call-template>
                  </xsl:if>
               </xsl:otherwise>
            </xsl:choose>
         </xsl:when>
         <!-- Derivation by restriction on simple content -->
         <xsl:when test="$type/xsd:simpleContent/xsd:restriction">
            <!-- Print out simple type constraints-->
               <xsl:text> </xsl:text>
               <xsl:apply-templates select="$type/xsd:simpleContent" mode="sample">
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:apply-templates>
               <xsl:text> </xsl:text>
         </xsl:when>
         <!-- Derivation by extension on simple content -->
         <xsl:when test="$type/xsd:simpleContent/xsd:extension">
            <!-- Print out base type name -->
               <xsl:text> </xsl:text>
               <xsl:call-template name="PrintTypeRef">
                  <xsl:with-param name="ref" select="$type/xsd:simpleContent/xsd:extension/@base"/>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:call-template>
               <xsl:text> </xsl:text>
         </xsl:when>
         <!-- No derivation: complex type definition -->
         <xsl:when test="local-name($type)='complexType'">
            <!-- Print out content from this type -->
            <xsl:call-template name="PrintSampleParticleList">
               <xsl:with-param name="list" select="$type"/>
               <xsl:with-param name="margin" select="$margin"/>
               <xsl:with-param name="isInherited" select="$isInherited"/>
               <xsl:with-param name="isNewField" select="$isNewField"/>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               <xsl:with-param name="typeList" select="concat($typeList, '*', $type/@name, '+')"/>
		<xsl:with-param name="parentGroups" select="$parentGroups"/>
            </xsl:call-template>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Prints out sample XML instances from a list of 
     element particle.
     Param(s):
            list (Node) required
                Node containing list of element particles
            margin (nonNegativeInteger) optional
                Number of 'em' to indent from left
            isInherited (boolean) optional
                If true, display elements using 'inherited' CSS class.
            isNewField (boolean) optional
                If true, display elements using 'newFields' CSS class.
            schemaLoc (String) optional
                Schema file containing this particle list; 
                if in current schema, 'schemaLoc' is set to 'this'.
            typeList (String) optional
                List of types in this call chain. Name of type starts
                with '*', and ends with '+'. (Used to prevent infinite
                recursive loop.)
     -->
   <xsl:template name="PrintSampleParticleList">
      <xsl:param name="list"/>
      <xsl:param name="margin">0</xsl:param>
      <xsl:param name="isInherited">false</xsl:param>
      <xsl:param name="isNewField">false</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>
      <xsl:param name="parentGroups"/>

      <xsl:if test="$list">
         <xsl:apply-templates select="$list/xsd:group | $list/xsd:sequence | $list/xsd:choice | $list/xsd:all | $list/xsd:element" mode="sample">
            <xsl:with-param name="margin" select="$margin"/>
            <xsl:with-param name="isInherited" select="$isInherited"/>
            <xsl:with-param name="isNewField" select="$isNewField"/>
            <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            <xsl:with-param name="typeList" select="$typeList"/>
	    <xsl:with-param name="parentGroups" select="$parentGroups"/>
         </xsl:apply-templates>
      </xsl:if>
   </xsl:template>

   <!--
     Prints out the constraints of simple content
     to be displayed within a sample XML instance.
     Param(s):
            simpleContent (Node) required
                Node containing with the simple content
            schemaLoc (String) optional
                Schema file containing these simple constraints; 
                if in current schema, 'schemaLoc' is set to 'this'.
     -->
   <xsl:template name="PrintSampleSimpleConstraints">
      <xsl:param name="simpleContent"/>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>

      <xsl:choose>
         <!-- Derivation by restriction -->
         <xsl:when test="$simpleContent/xsd:restriction">
            <xsl:call-template name="PrintSampleSimpleRestriction">
               <xsl:with-param name="restriction" select="$simpleContent/xsd:restriction"/>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               <xsl:with-param name="typeList" select="$typeList"/>
            </xsl:call-template>
         </xsl:when>
         <!-- Derivation by list -->
         <xsl:when test="$simpleContent/xsd:list">
            <xsl:choose>
               <xsl:when test="$simpleContent/xsd:list/@itemType">
                  <xsl:text>list of: </xsl:text>
                  <xsl:call-template name="PrintTypeRef">
                     <xsl:with-param name="ref" select="$simpleContent/xsd:list/@itemType"/>
                     <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                  </xsl:call-template>
               </xsl:when>
               <xsl:otherwise>
                  <xsl:text>list of: [ </xsl:text>
                  <xsl:call-template name="PrintSampleSimpleConstraints">
                     <xsl:with-param name="simpleContent" select="$simpleContent/xsd:list/xsd:simpleType"/>
                     <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
                  </xsl:call-template>
                  <xsl:text> ]</xsl:text>
               </xsl:otherwise>
            </xsl:choose>
         </xsl:when>
         <!-- Derivation by union -->
         <xsl:when test="$simpleContent/xsd:union">
            <xsl:text>union of: [ </xsl:text>

            <xsl:variable name="hasMemberTypes">
               <xsl:if test="normalize-space($simpleContent/xsd:union/@memberTypes)!=''">
                  <xsl:text>true</xsl:text>
               </xsl:if>
            </xsl:variable>
            <xsl:if test="$hasMemberTypes='true'">
               <xsl:call-template name="PrintWhitespaceList">
                  <xsl:with-param name="value" select="$simpleContent/xsd:union/@memberTypes"/>
                  <xsl:with-param name="compType">type</xsl:with-param>
                  <xsl:with-param name="separator">,</xsl:with-param>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:call-template>
            </xsl:if>
            <xsl:for-each select="$simpleContent/xsd:union/xsd:simpleType">
               <xsl:if test="position()!=1 or $hasMemberTypes='true'">
                  <xsl:text>, </xsl:text>
               </xsl:if>
               <xsl:text>[ </xsl:text>
               <xsl:call-template name="PrintSampleSimpleConstraints">
                  <xsl:with-param name="simpleContent" select="."/>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:call-template>
               <xsl:text> ]</xsl:text>
            </xsl:for-each>

            <xsl:text> ]</xsl:text>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Prints out the constraints of simple content
     derived by restriction, which is to be displayed 
     within a sample XML instance.
     Param(s):
            restriction (Node) required
                Node containing with the restriction
            schemaLoc (String) optional
                Schema file containing this restriction element; 
                if in current schema, 'schemaLoc' is set to 'this'.
     -->
   <xsl:template name="PrintSampleSimpleRestriction">
      <xsl:param name="restriction"/>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="typeList"/>

      <xsl:variable name="typeName" select="$restriction/parent::xsd:simpleType/@name"/>

      <!-- Print out base type info -->
      <xsl:choose>
         <!-- Circular type hierarchy -->
         <xsl:when test="$typeName != '' and contains($typeList, concat('*', $typeName, '+'))">
            <xsl:call-template name="HandleError">
               <xsl:with-param name="isTerminating">false</xsl:with-param>
               <xsl:with-param name="errorMsg">
                  <xsl:text>Circular type reference to '</xsl:text>
                  <xsl:value-of select="$typeName"/>
                  <xsl:text>' in type hierarchy.</xsl:text>
               </xsl:with-param>
            </xsl:call-template>
         </xsl:when>
         <!-- Locally-defined base type -->
         <xsl:when test="$restriction/xsd:simpleType">
            <xsl:call-template name="PrintSampleSimpleConstraints">
               <xsl:with-param name="simpleContent" select="$restriction/xsd:simpleType"/>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               <xsl:with-param name="typeList" select="$typeList"/>
            </xsl:call-template>
         </xsl:when>
         <!-- Base type reference -->
         <xsl:when test="$restriction">
            <xsl:variable name="baseTypeRef" select="$restriction/@base"/>
            <xsl:variable name="baseTypeName">
               <xsl:call-template name="GetRefName">
                  <xsl:with-param name="ref" select="$baseTypeRef"/>
               </xsl:call-template>
            </xsl:variable>
            <xsl:variable name="baseTypeNS">
               <xsl:call-template name="GetRefNS">
                  <xsl:with-param name="ref" select="$baseTypeRef"/>
               </xsl:call-template>
            </xsl:variable>
            <!-- Write out reference to base type -->
            <xsl:call-template name="PrintTypeRef">
               <xsl:with-param name="ref" select="$baseTypeRef"/>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:when>
      </xsl:choose>

      <!-- Regular Expression Pattern -->
      <xsl:variable name="pattern">
         <xsl:call-template name="PrintPatternFacet">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
      </xsl:variable>
      <!-- Range -->
      <xsl:variable name="range">
         <xsl:call-template name="PrintRangeFacets">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
      </xsl:variable>
      <!-- Length -->
      <xsl:variable name="length">
         <xsl:call-template name="PrintLengthFacets">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
      </xsl:variable>

      <!-- Print out facets -->
      <xsl:if test="$restriction/xsd:enumeration">
         <xsl:text> (</xsl:text>
         <xsl:call-template name="PrintEnumFacets">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
         <xsl:text>)</xsl:text>
      </xsl:if>
      <xsl:if test="$pattern !=''">
         <xsl:text> (</xsl:text>
         <xsl:copy-of select="$pattern"/>
         <xsl:text>)</xsl:text>
      </xsl:if>
      <xsl:if test="$range !=''">
         <xsl:text> (</xsl:text>
         <xsl:copy-of select="$range"/>
         <xsl:text>)</xsl:text>
      </xsl:if>
      <xsl:if test="$restriction/xsd:totalDigits">
         <xsl:text> (</xsl:text>
         <xsl:call-template name="PrintTotalDigitsFacet">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
         <xsl:text>)</xsl:text>
      </xsl:if>
      <xsl:if test="$restriction/xsd:fractionDigits">
         <xsl:text> (</xsl:text>
         <xsl:call-template name="PrintFractionDigitsFacet">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
         <xsl:text>)</xsl:text>
      </xsl:if>
      <xsl:if test="$length !=''">
         <xsl:text> (</xsl:text>
         <xsl:copy-of select="$length"/>
         <xsl:text>)</xsl:text>
      </xsl:if>
      <xsl:if test="$restriction/xsd:whiteSpace">
         <xsl:text> (</xsl:text>
         <xsl:call-template name="PrintWhitespaceFacet">
            <xsl:with-param name="simpleRestrict" select="$restriction"/>
         </xsl:call-template>
         <xsl:text>)</xsl:text>
      </xsl:if>
   </xsl:template>

   <!--
     Displays a schema attribute in the correct format
     for the Schema Component Representation table, e.g.
     tags are one color, and content are another.
     Param(s):
            attrName (String) required
                Name of attribute
            attrValue (Result Tree Fragment) required
                Value of attribute, which may be links
     -->
   <xsl:template name="DisplayAttr">
      <xsl:param name="attrName"/>
      <xsl:param name="attrValue"/>

      <xsl:text> </xsl:text>
         <xsl:value-of select="$attrName"/>
      <xsl:text>="</xsl:text>
      <xsl:if test="normalize-space($attrValue)!=''">
            <xsl:copy-of select="$attrValue"/>
      </xsl:if>
      <xsl:text>"</xsl:text>
   </xsl:template>

   <!--
     Displays attributes from a schema element, unless
     otherwise specified, in the correct format
     for the Schema Component Representation table, e.g.
     tags are one color, and content are another.
     Param(s):
            component (Node) required
                Schema element whose attributes are to be displayed
            attrsNotToDisplay (String) required
                List of attributes not to be displayed
                Each attribute name should prepended with '*'
                and appended with '+'
     -->
   <xsl:template name="DisplayOtherAttributes">
      <xsl:param name="component"/>
      <xsl:param name="attrsNotToDisplay"/>

      <xsl:for-each select="$component/attribute::*">
         <xsl:variable name="attrName" select="local-name(.)"/>
         <xsl:if test="not(contains($attrsNotToDisplay, concat('*', $attrName, '+')))">
            <xsl:call-template name="DisplayAttr">
               <xsl:with-param name="attrName" select="normalize-space($attrName)"/>
               <xsl:with-param name="attrValue" select="normalize-space(.)"/>
            </xsl:call-template>
         </xsl:if>
      </xsl:for-each>
   </xsl:template>


   <!-- ******** XML Pretty Printer ******** -->

   <!--
     Puts XHTML elements into the result.
     -->
   <xsl:template match="html:*" mode="html">
      <xsl:element name="{local-name(.)}">
         <xsl:for-each select="@*">
            <xsl:copy-of select="."/>
         </xsl:for-each>
         <xsl:apply-templates select="* | text()" mode="html"/>
      </xsl:element>
   </xsl:template>

   <!--
     Displays non-XHTML elements found within XHTML elements.
     -->
   <xsl:template match="*" mode="html">
      <xsl:call-template name="WriteElement">
         <xsl:with-param name="element" select="."/>
         <xsl:with-param name="mode">html</xsl:with-param>
      </xsl:call-template>
   </xsl:template>

   <!--
     Displays text node.
     -->
   <xsl:template match="text()" mode="html">
      <xsl:value-of select="."/>
   </xsl:template>

   <!--
     Displays an arbitrary XML element.
     -->
   <xsl:template match="*" mode="xpp">
      <code>
         <xsl:call-template name="WriteElement">
            <xsl:with-param name="element" select="."/>
            <xsl:with-param name="mode">xpp</xsl:with-param>
         </xsl:call-template>
      </code>
   </xsl:template>

   <!--
     Displays an arbitrary XML text node.
     -->
   <xsl:template match="text()" mode="xpp">
      <xsl:value-of select="."/>
   </xsl:template>

   <!--
     Displays an XML comment.
     -->
   <xsl:template match="comment()" mode="xpp">
      <div class="comment">
         <xsl:text>&lt;--</xsl:text>
         <xsl:value-of select="."/>
         <xsl:text>--&gt;</xsl:text>
      </div>
   </xsl:template>

   <!--
     Displays an XML element in the documentation, e.g.
     tags are escaped.
     Param(s):
            element (Node) required
                XML element to display
            mode (xpp|html) required
                Which mode to invoke for child elements
     -->
   <xsl:template name="WriteElement">
      <xsl:param name="element"/>
      <xsl:param name="mode">xpp</xsl:param>

      <!-- Start Tag -->
      <xsl:text>&lt;</xsl:text>
      <xsl:value-of select="local-name($element)"/>
      <!-- Attributes -->
      <xsl:for-each select="$element/@*">
         <xsl:text> </xsl:text>
         <xsl:value-of select="name(.)"/>
         <xsl:text>="</xsl:text>
         <xsl:value-of select="."/>
         <xsl:text>"</xsl:text>
      </xsl:for-each>

      <xsl:choose>
         <xsl:when test="$element/* | $element/text()">
            <!-- Close Start Tag -->
            <xsl:text>> </xsl:text>
            <!-- Content -->
            <xsl:choose>
               <xsl:when test="$mode!='xpp'">
                  <xsl:apply-templates select="$element/* | $element/text()" mode="html"/>
               </xsl:when>
               <xsl:otherwise>
                  <div >
                     <xsl:apply-templates select="$element/* | $element/text()" mode="xpp"/>
                  </div>
               </xsl:otherwise>
            </xsl:choose>
            <!-- End Tag -->
            <xsl:text>&lt;/</xsl:text>
            <xsl:value-of select="local-name($element)"/>
            <xsl:text>></xsl:text>
         </xsl:when>
         <xsl:otherwise>
            <!-- Close Start Tag -->
            <xsl:text>/></xsl:text>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>


   <!-- ******** Templates for Handling References ******** -->

   <!--
     Prints out a reference to a term in the glossary section.
     Param(s):
            code (String) required
              Unique ID of glossary term
            term (String) optional
              Glossary term
     -->
   <xsl:template name="PrintGlossaryTermRef">
      <xsl:param name="code"/>
      <xsl:param name="term"/>

      <xsl:choose>
         <xsl:when test="$code !=''">
            <a title="Look up '{$term}' in glossary">
               <xsl:choose>
                  <xsl:when test="$term!=''">
                     <xsl:value-of select="$term"/>
                  </xsl:when>
                  <xsl:otherwise>
                     <xsl:text>[term]</xsl:text>
                  </xsl:otherwise>
               </xsl:choose>
            </a>
         </xsl:when>
         <xsl:otherwise>
            <xsl:value-of select="$term"/>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Generates a link to an attribute.
     Param(s):
            name (String) optional
                Name of attribute
            ref (String) optional
                Reference to attribute
            (One of 'name' and 'ref' must be provided.)
            schemaLoc (String) optional
                Schema file containing this attribute reference
                if in current schema, 'schemaLoc' is set to 'this'
     -->
   <xsl:template name="PrintAttributeRef">
      <xsl:param name="name"/>
      <xsl:param name="ref"/>
      <xsl:param name="schemaLoc">this</xsl:param>

      <xsl:choose>
         <xsl:when test="$name!=''">
            <xsl:call-template name="PrintCompName">
               <xsl:with-param name="name" select="$name"/>
               <xsl:with-param name="compType">attribute</xsl:with-param>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:when>
         <xsl:when test="$ref!=''">
            <xsl:call-template name="PrintCompRef">
               <xsl:with-param name="ref" select="$ref"/>
               <xsl:with-param name="compType">attribute</xsl:with-param>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Generates a link to an attribute group.
     Param(s):
            name (String) optional
                Name of attribute group
            ref (String) optional
                Reference to attribute group
            (One of 'name' and 'ref' must be provided.)
            schemaLoc (String) optional
                Schema file containing this attribute group reference
                if in current schema, 'schemaLoc' is set to 'this'
     -->
   <xsl:template name="PrintAttributeGroupRef">
      <xsl:param name="name"/>
      <xsl:param name="ref"/>
      <xsl:param name="schemaLoc">this</xsl:param>

      <xsl:choose>
         <xsl:when test="$name!=''">
            <xsl:call-template name="PrintCompName">
               <xsl:with-param name="name" select="$name"/>
               <xsl:with-param name="compType">attribute group</xsl:with-param>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:when>
         <xsl:when test="$ref!=''">
            <xsl:call-template name="PrintCompRef">
               <xsl:with-param name="ref" select="$ref"/>
               <xsl:with-param name="compType">attribute group</xsl:with-param>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Generates a link to an element.
     Param(s):
            name (String) optional
                Name of element
            ref (String) optional
                Reference to element
            (One of 'name' and 'ref' must be provided.)
            schemaLoc (String) optional
                Schema file containing this element reference
                if in current schema, 'schemaLoc' is set to 'this'
     -->
   <xsl:template name="PrintElementRef">
      <xsl:param name="name"/>
      <xsl:param name="ref"/>
      <xsl:param name="schemaLoc">this</xsl:param>

      <xsl:choose>
         <xsl:when test="$name!=''">
            <xsl:call-template name="PrintCompName">
               <xsl:with-param name="name" select="$name"/>
               <xsl:with-param name="compType">element</xsl:with-param>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:when>
         <xsl:when test="$ref!=''">
            <xsl:call-template name="PrintCompRef">
               <xsl:with-param name="ref" select="$ref"/>
               <xsl:with-param name="compType">element</xsl:with-param>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Generates a link to a group.
     Param(s):
            name (String) optional
                Name of group
            ref (String) optional
                Reference to group
            (One of 'name' and 'ref' must be provided.)
            schemaLoc (String) optional
                Schema file containing this group reference
                if in current schema, 'schemaLoc' is set to 'this'
     -->
   <xsl:template name="PrintGroupRef">
      <xsl:param name="name"/>
      <xsl:param name="ref"/>
      <xsl:param name="schemaLoc">this</xsl:param>

      <xsl:choose>
         <xsl:when test="$name!=''">
            <xsl:call-template name="PrintCompName">
               <xsl:with-param name="name" select="$name"/>
               <xsl:with-param name="compType">group</xsl:with-param>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:when>
         <xsl:when test="$ref!=''">
            <xsl:call-template name="PrintCompRef">
               <xsl:with-param name="ref" select="$ref"/>
               <xsl:with-param name="compType">group</xsl:with-param>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Generates a link to a key/uniqueness constraint.
     Param(s):
            name (String) optional
                Name of key/uniqueness constraint
            ref (String) optional
                Reference to key/uniqueness constraint
            (One of 'name' and 'ref' must be provided.)
            schemaLoc (String) optional
                Schema file containing this key/uniqueness constraint
                reference; if in current schema, 'schemaLoc' is set 
                to 'this'
     -->
   <xsl:template name="PrintKeyRef">
      <xsl:param name="name"/>
      <xsl:param name="ref"/>
      <xsl:param name="schemaLoc">this</xsl:param>

      <xsl:choose>
         <xsl:when test="$name!=''">
            <xsl:call-template name="PrintCompName">
               <xsl:with-param name="name" select="$name"/>
               <xsl:with-param name="compType">uniqueness/key constraint</xsl:with-param>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:when>
         <xsl:when test="$ref!=''">
            <xsl:call-template name="PrintCompRef">
               <xsl:with-param name="ref" select="$ref"/>
               <xsl:with-param name="compType">uniqueness/key constraint</xsl:with-param>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Generates a link to a type.
     Param(s):
            name (String) optional
                Name of type
            ref (String) optional
                Reference to type
            (One of 'name' and 'ref' must be provided.)
            schemaLoc (String) optional
                Schema file containing this type reference'
                if in current schema, 'schemaLoc' is set 
                to 'this'
     -->
   <xsl:template name="PrintTypeRef">
      <xsl:param name="name"/>
      <xsl:param name="ref"/>
      <xsl:param name="schemaLoc">this</xsl:param>

      <xsl:choose>
         <xsl:when test="$name!=''">
               <xsl:call-template name="PrintCompName">
                  <xsl:with-param name="name" select="$name"/>
                  <xsl:with-param name="compType">type</xsl:with-param>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:call-template>
         </xsl:when>
         <xsl:when test="$ref!=''">
               <xsl:call-template name="PrintCompRef">
                  <xsl:with-param name="ref" select="$ref"/>
                  <xsl:with-param name="compType">type</xsl:with-param>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:call-template>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Prints out a link to a schema component's section.
     Param(s):
            baseFile (String) optional
                Documentation file of schema containing this 
                component.
                If this component belongs to the current schema, 
                omit this variable.
                If this component is from an included or imported 
                schema, provide this variable.
            name (String) required
                Name of schema component
            compType (String) required
                Type of schema component
            errMsg (String) optional
                Sentence fragment.
                If specified, link will open up an alert box with 
                an error message. For example, if 'errMsg' was set 
                to "could not be found", 'name' was "x", and
                'compType' was "type", the error message would be:
                "x" type definition could not be found.
                The sentence fragment should not:
                -start with a capital letter.
                -have a space in front
                -end with a period.
            schemaLoc (String) optional
                Schema file containing this component;
                if in current schema, 'schemaLoc' is set to 'this'
     -->
   <xsl:template name="PrintCompName">
      <xsl:param name="baseFile"/>
      <xsl:param name="name"/>
      <xsl:param name="compType"/>
      <xsl:param name="schemaLoc">this</xsl:param>
      <xsl:param name="errMsg"/>

      <!-- Get correct terminology for statements -->
      <xsl:variable name="noun">
         <xsl:choose>
            <xsl:when test="$compType='element' or $compType='attribute'">
               <xsl:text>declaration</xsl:text>
            </xsl:when>
            <xsl:otherwise>
               <xsl:text>definition</xsl:text>
            </xsl:otherwise>
         </xsl:choose>
      </xsl:variable>

      <!-- Get prefix to use in anchor name. -->
      <xsl:variable name="compPrefix">
         <xsl:choose>
            <xsl:when test="$compType='attribute'">
            </xsl:when>
            <xsl:when test="$compType='attribute group'">
            </xsl:when>
            <xsl:when test="$compType='element'">
            </xsl:when>
            <xsl:when test="$compType='group'">
            </xsl:when>
            <xsl:when test="$compType='type'">
            </xsl:when>
            <xsl:when test="$compType='uniqueness/key constraint'">
            </xsl:when>
         </xsl:choose>
      </xsl:variable>

      <!-- Get base URI. -->
      <xsl:variable name="baseURI">
         <xsl:choose>
            <xsl:when test="$baseFile!=''">
               <xsl:value-of select="$baseFile"/>
            </xsl:when>
         </xsl:choose>
      </xsl:variable>

      <!-- Generate message. -->
      <xsl:variable name="title">
         <xsl:choose>
            <!-- Error message was provided. -->
            <xsl:when test="$errMsg!=''">
               <xsl:text>"</xsl:text><xsl:value-of select="$name"/><xsl:text>" </xsl:text>
               <xsl:value-of select="$compType"/><xsl:text> </xsl:text>
               <xsl:value-of select="$noun"/><xsl:text> </xsl:text>
               <xsl:value-of select="$errMsg"/>
            </xsl:when>
            <!-- There exists a link to the schema component's
                 documentation. -->
            <xsl:otherwise>
               <xsl:text>Jump to "</xsl:text>
               <xsl:value-of select="$name"/>
               <xsl:text>" </xsl:text>
               <xsl:value-of select="$compType"/>
               <xsl:text> </xsl:text>
               <xsl:value-of select="$noun"/>
               <!-- External link -->
               <xsl:if test="normalize-space($baseURI)!=''">
                  <xsl:text>(located in external schema documentation)</xsl:text>
               </xsl:if>
            </xsl:otherwise>
         </xsl:choose>
         <xsl:text>.</xsl:text>
      </xsl:variable>

      <!-- Generate href link -->
      <xsl:variable name="link">
         <xsl:choose>
            <!-- Error message was provided. -->
            <xsl:when test="$errMsg!=''">
               <xsl:text>javascript:void(0)</xsl:text>
            </xsl:when>
            <!-- There exists a link to the schema component's
                 documentation. -->
            <xsl:otherwise>
               <!-- Base URI -->
               <xsl:value-of select="normalize-space($baseURI)"/>
               <!-- Anchor within URI -->
               <xsl:value-of select="concat('#',normalize-space($compPrefix),normalize-space($name))"/>
            </xsl:otherwise>
         </xsl:choose>
      </xsl:variable>

      <a title="{$title}" href="{$link}">
         <!-- External link -->
         <xsl:if test="normalize-space($baseURI)!=''">
            <xsl:attribute name="class">externalLink</xsl:attribute>
         </xsl:if>

         <!-- Error message was provided. -->
         <xsl:if test="$errMsg!=''">
            <xsl:attribute name="onclick">
               <xsl:text>alert('</xsl:text>
               <xsl:value-of select="$title"/>
               <xsl:text>');</xsl:text>
            </xsl:attribute>
         </xsl:if>

         <xsl:value-of select="$name"/>
      </a>
   </xsl:template>

   <!--
     Prints out a reference to a schema component.
     This template will try to work out which schema that this
     component belongs to and print out the appropriate link.
     component.
     It will also print out the namespace prefix given
     in the reference.
     Param(s):
            ref (String) required
                Reference to schema component
            compType (String) required
                Type of schema component
            schemaLoc (String) optional
                Schema file containing this component reference;
                if in current schema, 'schemaLoc' is set to 'this'
     -->
   <xsl:template name="PrintCompRef">
      <xsl:param name="ref"/>
      <xsl:param name="compType"/>
      <xsl:param name="schemaLoc">this</xsl:param>

      <!-- Get correct terminology for statements -->
      <xsl:variable name="noun">
         <xsl:choose>
            <xsl:when test="$compType='element' or $compType='attribute'">
               <xsl:text>declaration</xsl:text>
            </xsl:when>
            <xsl:otherwise>
               <xsl:text>definition</xsl:text>
            </xsl:otherwise>
         </xsl:choose>
      </xsl:variable>

      <!-- Get local name -->
      <xsl:variable name="refName">
         <xsl:call-template name="GetRefName">
            <xsl:with-param name="ref" select="$ref"/>
         </xsl:call-template>
      </xsl:variable>

      <!-- Get prefix -->
      <xsl:variable name="refPrefix">
         <xsl:call-template name="GetRefPrefix">
            <xsl:with-param name="ref" select="$ref"/>
         </xsl:call-template>
      </xsl:variable>

      <!-- Get prefix of this schema's target namespace -->
      <xsl:variable name="tnPrefix">
         <xsl:call-template name="GetThisPrefix"/>
      </xsl:variable>

      <!-- Get file location of the schema component that is
           being referenced. -->
      <xsl:variable name="compLoc">
         <xsl:call-template name="FindComponent">
            <xsl:with-param name="ref" select="$ref"/>
            <xsl:with-param name="compType" select="$compType"/>
         </xsl:call-template>
      </xsl:variable>

      <!-- Print local name -->
      <xsl:choose>
         <!-- Component from XML Schema's or XML's namespace -->
         <xsl:when test="normalize-space($compLoc)='xsd' or normalize-space($compLoc)='xml'">
            <xsl:value-of select="$refName"/>
         </xsl:when>
         <!-- Component found in this schema. -->
         <xsl:when test="normalize-space($compLoc)='this'">
            <xsl:call-template name="PrintCompName">
               <xsl:with-param name="name" select="$refName"/>
               <xsl:with-param name="compType" select="$compType"/>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:when>
         <!-- Component not found. -->
         <xsl:when test="normalize-space($compLoc)='' or normalize-space($compLoc)='none'">
            <xsl:call-template name="PrintCompName">
               <xsl:with-param name="name" select="$refName"/>
               <xsl:with-param name="compType" select="$compType"/>
               <xsl:with-param name="errMsg">could not be found</xsl:with-param>
            </xsl:call-template>
         </xsl:when>
         <!-- Component found in an external schema. -->
         <xsl:otherwise>

            <xsl:call-template name="PrintCompName">
               <xsl:with-param name="name" select="$refName"/>
               <xsl:with-param name="compType" select="$compType"/>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>


   <!-- ******** Templates for Finding Components in Different
             Schema documents ******** -->

   <!-- Special key words: xml, xsd, this, none -->
   <xsl:template name="FindComponent">
      <xsl:param name="ref"/>
      <xsl:param name="compType"/>

      <!-- Get local name -->
      <xsl:variable name="refName">
         <xsl:call-template name="GetRefName">
            <xsl:with-param name="ref" select="$ref"/>
         </xsl:call-template>
      </xsl:variable>

      <!-- Get prefix -->
      <xsl:variable name="refPrefix">
         <xsl:call-template name="GetRefPrefix">
            <xsl:with-param name="ref" select="$ref"/>
         </xsl:call-template>
      </xsl:variable>

      <!-- Get prefix of this schema's target namespace -->
      <xsl:variable name="tnPrefix">
         <xsl:call-template name="GetThisPrefix"/>
      </xsl:variable>

      <!-- Get prefix of XML Schema -->
      <xsl:variable name="xsdPrefix">
         <xsl:call-template name="GetXSDPrefix"/>
      </xsl:variable>

      <xsl:choose>
         <!-- Schema component from XML Schema's namespace,
              unless this schema is for XML Schema -->
         <xsl:when test="$refPrefix=$xsdPrefix and $xsdPrefix!=$tnPrefix">
            <xsl:text>xsd</xsl:text>
         </xsl:when>
         <!-- Schema component from XML's namespace -->
         <xsl:when test="$refPrefix='xml'">
            <xsl:text>xml</xsl:text>
         </xsl:when>
         <!-- Schema component from current schema's namespace -->
         <xsl:when test="$refPrefix=$tnPrefix">
            <xsl:call-template name="FindComponentInSchema">
               <xsl:with-param name="name" select="$refName"/>
               <xsl:with-param name="compType" select="$compType"/>
               <xsl:with-param name="schema" select="/xsd:schema"/>
               <xsl:with-param name="schemaFileLoc">this</xsl:with-param>
            </xsl:call-template>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <xsl:template name="FindComponentInSchema">
      <xsl:param name="name"/>
      <xsl:param name="compType"/>
      <xsl:param name="schema"/>
      <xsl:param name="schemaFileLoc"/>
      <xsl:param name="schemasSearched"/>

      <!-- Don't examine this schema if we've already
           searched it. Prevents infinite recursion.
           Also check if schema actually exists. -->
      <xsl:if test="$schema and not(contains($schemasSearched, concat('*', $schemaFileLoc, '+')))">
         <!-- Find out if the component is in this schema -->
         <xsl:variable name="thisResult">
            <xsl:call-template name="IsComponentInSchema">
               <xsl:with-param name="name" select="$name"/>
               <xsl:with-param name="compType" select="$compType"/>
               <xsl:with-param name="schema" select="$schema"/>
            </xsl:call-template>
         </xsl:variable>

         <xsl:choose>
            <!-- Component found -->
            <xsl:when test="normalize-space($thisResult)='true'">
               <xsl:value-of select="$schemaFileLoc"/>
            </xsl:when>
         </xsl:choose>
      </xsl:if>
   </xsl:template>

   <xsl:template name="IsComponentInSchema">
      <xsl:param name="name"/>
      <xsl:param name="compType"/>
      <xsl:param name="schema"/>

      <xsl:choose>
         <!-- Schema not found. -->
         <xsl:when test="not($schema)">
            <xsl:text>false</xsl:text>
         </xsl:when>
         <!-- Search for attribute declaration. -->
         <xsl:when test="$compType='attribute' and $schema/xsd:attribute[@name=$name]">
            <xsl:text>true</xsl:text>
         </xsl:when>
         <!-- Search for attribute group definition. -->
         <xsl:when test="$compType='attribute group' and ($schema/xsd:attributeGroup[@name=$name] or $schema/xsd:redefine/xsd:attributeGroup[@name=$name])">
            <xsl:text>true</xsl:text>
         </xsl:when>
         <!-- Search for element declaration. -->
         <xsl:when test="$compType='element' and $schema/xsd:element[@name=$name]">
            <xsl:text>true</xsl:text>
         </xsl:when>
         <!-- Search for group definition. -->
         <xsl:when test="$compType='group' and ($schema/xsd:group[@name=$name] or $schema/xsd:redefine/xsd:group[@name=$name])">
            <xsl:text>true</xsl:text>
         </xsl:when>
         <!-- Search for complex type definition. -->
         <xsl:when test="($compType='type' or $compType='complex type') and ($schema/xsd:complexType[@name=$name] or $schema/xsd:redefine/xsd:complexType[@name=$name])">
            <xsl:text>true</xsl:text>
         </xsl:when>
         <!-- Search for simple type definition. -->
         <xsl:when test="($compType='type' or $compType='simple type') and ($schema/xsd:simpleType[@name=$name] or $schema/xsd:redefine/xsd:simpleType[@name=$name])">
            <xsl:text>true</xsl:text>
         </xsl:when>
         <!-- Search for uniqueness/key constraint definition. -->
         <xsl:when test="$compType='uniqueness/key constraint' and ($schema//xsd:element/xsd:key[@name=$name] or $schema//xsd:element/xsd:unique[@name=$name])">
            <xsl:text>true</xsl:text>
         </xsl:when>
         <!-- Component not found. -->
         <xsl:otherwise>
            <xsl:text>false</xsl:text>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <xsl:template name="FindComponentInIncludedSchemas">
      <xsl:param name="schema"/>
      <xsl:param name="name"/>
      <xsl:param name="compType"/>
      <xsl:param name="schemasSearched"/>
      <xsl:param name="index">1</xsl:param>

      <xsl:if test="count($schema/xsd:include) &gt;= number($index)">
         <!-- Get the 'schemaLocation' attribute of the 'include'
              element in this schema at position, 'index'. -->
         <xsl:variable name="schemaLoc" select="$schema/xsd:include[position()=$index]/@schemaLocation"/>

         <xsl:variable name="thisResult">
            <!-- Search for the component in the current 
                 included schema. -->
            <xsl:call-template name="FindComponentInSchema">
               <xsl:with-param name="name" select="$name"/>
               <xsl:with-param name="compType" select="$compType"/>
               <xsl:with-param name="schema" select="document($schemaLoc)/xsd:schema"/>
               <xsl:with-param name="schemaFileLoc" select="$schemaLoc"/>
               <xsl:with-param name="schemasSearched" select="$schemasSearched"/>
            </xsl:call-template>
         </xsl:variable>

         <xsl:choose>
            <!-- Component was found, so return result. -->
            <xsl:when test="normalize-space($thisResult)!='' and normalize-space($thisResult)!='none'">
               <xsl:value-of select="$thisResult"/>
            </xsl:when>
            <!-- Component was not found, so keep on searching. -->
            <xsl:otherwise>
               <!-- Examine other included schemas in this schema -->
               <xsl:call-template name="FindComponentInIncludedSchemas">
                  <xsl:with-param name="schema" select="$schema"/>
                  <xsl:with-param name="name" select="$name"/>
                  <xsl:with-param name="compType" select="$compType"/>
                  <xsl:with-param name="index" select="number($index)+1"/>
                  <xsl:with-param name="schemasSearched" select="concat($schemasSearched, '*', $schemaLoc, '+')"/>
               </xsl:call-template>
            </xsl:otherwise>
         </xsl:choose>
      </xsl:if>
   </xsl:template>

   <xsl:template name="FindComponentInRedefinedSchemas">
      <xsl:param name="schema"/>
      <xsl:param name="name"/>
      <xsl:param name="compType"/>
      <xsl:param name="schemasSearched"/>
      <xsl:param name="index">1</xsl:param>

      <xsl:if test="count($schema/xsd:redefine) &gt;= number($index)">
         <!-- Get the 'schemaLocation' attribute of the 'redefine'
              element in this schema at position, 'index'. -->
         <xsl:variable name="schemaLoc" select="$schema/xsd:redefine[position()=$index]/@schemaLocation"/>

         <xsl:variable name="thisResult">
            <!-- Search for the component in the current 
                 redefined schema. -->
            <xsl:call-template name="FindComponentInSchema">
               <xsl:with-param name="name" select="$name"/>
               <xsl:with-param name="compType" select="$compType"/>
               <xsl:with-param name="schema" select="document($schemaLoc)/xsd:schema"/>
               <xsl:with-param name="schemaFileLoc" select="$schemaLoc"/>
               <xsl:with-param name="schemasSearched" select="$schemasSearched"/>
            </xsl:call-template>
         </xsl:variable>

         <xsl:choose>
            <!-- Component was found, so return result. -->
            <xsl:when test="normalize-space($thisResult)!='' and normalize-space($thisResult)!='none'">
               <xsl:value-of select="$thisResult"/>
            </xsl:when>
            <!-- Component was not found, so keep on searching. -->
            <xsl:otherwise>
               <!-- Examine other redefined schemas in this schema -->
               <xsl:call-template name="FindComponentInRedefinedSchemas">
                  <xsl:with-param name="schema" select="$schema"/>
                  <xsl:with-param name="name" select="$name"/>
                  <xsl:with-param name="compType" select="$compType"/>
                  <xsl:with-param name="index" select="number($index)+1"/>
                  <xsl:with-param name="schemasSearched" select="concat($schemasSearched, '*', $schemaLoc, '+')"/>
               </xsl:call-template>
            </xsl:otherwise>
         </xsl:choose>
      </xsl:if>
   </xsl:template>

   <xsl:template name="FindComponentInImportedSchemas">
      <xsl:param name="schema"/>
      <xsl:param name="name"/>
      <xsl:param name="compType"/>
      <xsl:param name="compNS"/>
      <xsl:param name="schemasSearched"/>
      <xsl:param name="index">1</xsl:param>

      <xsl:if test="count($schema/xsd:import) &gt;= number($index)">
         <!-- Get the 'namespace' attribute of the 'import'
              element in this schema at position, 'index'. -->
         <xsl:variable name="schemaNS" select="$schema/xsd:import[position()=$index]/@namespace"/>
         <!-- Get the 'schemaLocation' attribute. -->
         <xsl:variable name="schemaLoc" select="$schema/xsd:import[position()=$index]/@schemaLocation"/>

         <xsl:variable name="thisResult">
            <!-- Check that the imported schema has the matching
                 namespace as the component that we're looking
                 for. -->
            <xsl:if test="normalize-space($compNS)=normalize-space($schemaNS)">
               <!-- Search for the component in the current 
                    imported schema. -->
               <xsl:call-template name="FindComponentInSchema">
                  <xsl:with-param name="name" select="$name"/>
                  <xsl:with-param name="compType" select="$compType"/>
                  <xsl:with-param name="schema" select="document($schemaLoc)/xsd:schema"/>
                  <xsl:with-param name="schemaFileLoc" select="$schemaLoc"/>
                  <xsl:with-param name="schemasSearched" select="$schemasSearched"/>
               </xsl:call-template>
            </xsl:if>
         </xsl:variable>

         <xsl:choose>
            <!-- Component was found, so return result. -->
            <xsl:when test="normalize-space($thisResult)!='' and normalize-space($thisResult)!='none'">
               <xsl:value-of select="$thisResult"/>
            </xsl:when>
            <!-- Component was not found, so keep on searching. -->
            <xsl:otherwise>
               <!-- Examine other included schemas in this schema -->
               <xsl:call-template name="FindComponentInImportedSchemas">
                  <xsl:with-param name="schema" select="$schema"/>
                  <xsl:with-param name="name" select="$name"/>
                  <xsl:with-param name="compType" select="$compType"/>
                  <xsl:with-param name="compNS" select="$compNS"/>
                  <xsl:with-param name="index" select="number($index)+1"/>
                  <xsl:with-param name="schemasSearched" select="concat($schemasSearched, '*', $schemaLoc, '+')"/>
               </xsl:call-template>
            </xsl:otherwise>
         </xsl:choose>
      </xsl:if>
   </xsl:template>


   <!-- ******** General Utility Templates ******** -->

   <!-- 
     Creates a box that can be opened and closed, such
     that the contents can be hidden away until a button
     is pressed.
     Param(s):
            id (String) required
              Unique ID of the 'div' box
            caption (String) required
              Text describing the contents of the box;
              it will always be shown even when the box
              is closed
            contents (String) required
              Contents of box, which may appear and disappear
              with the press of a button.
            anchor (String) optional
              Anchor, e.g. <a name="...", for this box
            styleClass (String) optional
              Additional CSS class for the entire collapseable box
            isOpened (String) optional
              Set to true if initially opened, and
              false if initially closed
     -->
   <xsl:template name="CollapseableBox">
      <xsl:param name="id"/>
      <xsl:param name="caption"/>
     <xsl:param name="contents"/>
      <xsl:param name="styleClass"/>
     <xsl:value-of select="$caption"/>
     <xsl:text>
</xsl:text>
     <xsl:call-template name="PrintHeaderLine">
       <xsl:with-param name="header" select="'&quot;'"/>
       <xsl:with-param name="length" select="string-length($caption)"/>
     </xsl:call-template>
     <xsl:text>

.. code:: xml

    </xsl:text>
     <xsl:copy-of select="$contents"/>
     <xsl:text>

</xsl:text>
   </xsl:template>

   <!--
     Returns the namespace of an attribute
     declaration or reference.
     Param(s):
            attribute (Node) required
                Attribute declaration or reference
     -->
   <xsl:template name="GetAttributeNS">
      <xsl:param name="attribute"/>

      <xsl:choose>
         <!-- Qualified local attribute declaration -->
         <xsl:when test="$attribute[@name] and (normalize-space(translate($attribute/@form, 'QUALIFED', 'qualifed'))='qualified' or normalize-space(translate(/xsd:schema/@attributeFormDefault, 'QUALIFED', 'qualifed'))='qualified')">
            <xsl:value-of select="/xsd:schema/@targetNamespace"/>
         </xsl:when>
         <!-- Reference to global attribute declaration -->
         <xsl:when test="$attribute[@ref]">
            <xsl:call-template name="GetRefNS">
               <xsl:with-param name="ref" select="$attribute/@ref"/>
            </xsl:call-template>
         </xsl:when>
         <!-- Otherwise, attribute has no namespace -->
      </xsl:choose>
   </xsl:template>

   <!--
     Returns the description that can be used in 
     headers for a schema component.
     Param(s):
            component (Node) required
                Schema component
     -->
   <xsl:template name="GetComponentDescription">
      <xsl:param name="component"/>

      <xsl:choose>
         <xsl:when test="local-name($component)='all'">
            <xsl:text>All Model Group</xsl:text>
         </xsl:when>
         <xsl:when test="local-name($component)='attribute'">
            <xsl:text>Attribute</xsl:text>
         </xsl:when>
         <xsl:when test="local-name($component)='attributeGroup'">
            <xsl:text>Attribute Group</xsl:text>
         </xsl:when>
         <xsl:when test="local-name($component)='choice'">
            <xsl:text>Choice Model Group</xsl:text>
         </xsl:when>
         <xsl:when test="local-name($component)='complexType'">
            <xsl:text>Complex Type</xsl:text>
         </xsl:when>
         <xsl:when test="local-name($component)='element'">
            <xsl:text>Element</xsl:text>
         </xsl:when>
         <xsl:when test="local-name($component)='group'">
            <xsl:text>Model Group</xsl:text>
         </xsl:when>
         <xsl:when test="local-name($component)='notation'">
            <xsl:text>Notation</xsl:text>
         </xsl:when>
         <xsl:when test="local-name($component)='sequence'">
            <xsl:text>Sequence Model Group</xsl:text>
         </xsl:when>
         <xsl:when test="local-name($component)='simpleType'">
            <xsl:text>Simple Type</xsl:text>
         </xsl:when>
         <xsl:otherwise>
            <xsl:call-template name="HandleError">
               <xsl:with-param name="isTerminating">true</xsl:with-param>
               <xsl:with-param name="errorMsg">
Unknown schema component, <xsl:value-of select="local-name($component)"/>.
               </xsl:with-param>
            </xsl:call-template>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Returns the unique identifier for a top-level schema
     component. Returns the string "schema" if the 'component' 
     is the root schema element.
     Param(s):
            component (Node) required
                Schema component
     -->
   <xsl:template name="GetComponentID">
      <xsl:param name="component"/>

      <xsl:choose>
         <xsl:when test="local-name($component)='schema'">
            <xsl:text>schema</xsl:text>
         </xsl:when>
         <xsl:otherwise>
            <xsl:variable name="componentPrefix">
               <xsl:call-template name="GetComponentPrefix">
                  <xsl:with-param name="component" select="$component"/>
               </xsl:call-template>
            </xsl:variable>
            <xsl:value-of select="concat($componentPrefix, $component/@name)"/>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Returns the prefix to add in front of a schema component
     name when generating anchor names.
     Param(s):
            component (Node) required
                Schema component
     -->
   <xsl:template name="GetComponentPrefix">
      <xsl:param name="component"/>

      <xsl:choose>
         <xsl:when test="local-name($component)='attribute'">
         </xsl:when>
         <xsl:when test="local-name($component)='attributeGroup'">
         </xsl:when>
         <xsl:when test="local-name($component)='complexType'">
         </xsl:when>
         <xsl:when test="local-name($component)='element'">
         </xsl:when>
         <xsl:when test="local-name($component)='group'">
         </xsl:when>
         <xsl:when test="local-name($component)='notation'">
         </xsl:when>
         <xsl:when test="local-name($component)='simpleType'">
         </xsl:when>
         <xsl:when test="local-name($component)='key' or local-name($component)='unique'">
         </xsl:when>
         <xsl:otherwise>
            <xsl:call-template name="HandleError">
               <xsl:with-param name="isTerminating">true</xsl:with-param>
               <xsl:with-param name="errorMsg">
Unknown schema component, <xsl:value-of select="local-name($component)"/>.
               </xsl:with-param>
            </xsl:call-template>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Returns a glossary term reference for the 
     schema component type, if applicable.
     Param(s):
            component (Node) required
                Schema component
     -->
   <xsl:template name="GetComponentTermRef">
      <xsl:param name="component"/>

      <xsl:choose>
         <xsl:when test="local-name($component)='notation'">
            <xsl:text>Notation</xsl:text>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Returns the namespace prefix of an attribute.
     Param(s):
            attribute (Node) required
                Attribute declaration or reference
     -->
   <xsl:template name="GetAttributePrefix">
      <xsl:param name="attribute"/>

      <xsl:choose>
         <!-- Element reference -->
         <xsl:when test="$attribute/@ref">
            <xsl:call-template name="GetRefPrefix">
               <xsl:with-param name="ref" select="$attribute/@ref"/>
            </xsl:call-template>
         </xsl:when>
         <!-- Global element declaration -->
         <xsl:when test="local-name($attribute/..)='schema'">
            <xsl:call-template name="GetThisPrefix"/>
         </xsl:when>
         <!-- Local element declaration -->
         <xsl:otherwise>
            <xsl:if test="($attribute/@form and normalize-space($attribute/@form)='qualified') or (/xsd:schema/@attributeFormDefault and normalize-space(/xsd:schema/@attributeFormDefault)='qualified')">
               <xsl:call-template name="GetThisPrefix"/>
            </xsl:if>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Returns the namespace prefix of an element.
     Param(s):
            element (Node) required
                Element declaration or reference
     -->
   <xsl:template name="GetElementPrefix">
      <xsl:param name="element"/>

      <xsl:choose>
         <!-- Element reference -->
         <xsl:when test="$element/@ref">
            <xsl:call-template name="GetRefPrefix">
               <xsl:with-param name="ref" select="$element/@ref"/>
            </xsl:call-template>
         </xsl:when>
         <!-- Global element declaration -->
         <xsl:when test="local-name($element/..)='schema'">
            <xsl:call-template name="GetThisPrefix"/>
         </xsl:when>
         <!-- Local element declaration -->
         <xsl:otherwise>
            <xsl:if test="($element/@form and normalize-space($element/@form)='qualified') or (/xsd:schema/@elementFormDefault and normalize-space(/xsd:schema/@elementFormDefault)='qualified')">
               <xsl:call-template name="GetThisPrefix"/>
            </xsl:if>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Returns the local name of a reference.
     Param(s):
            ref (String) required
                Reference
     -->
   <xsl:template name="GetRefName">
      <xsl:param name="ref"/>

      <xsl:choose>
         <xsl:when test="contains($ref, ':')">
            <!-- Ref has namespace prefix -->
            <xsl:value-of select="substring-after($ref, ':')"/>
         </xsl:when>
         <xsl:otherwise>
            <!-- Ref has no namespace prefix -->
            <xsl:value-of select="$ref"/>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Returns the namespace prefix of a reference.
     Param(s):
            ref (String) required
                Reference
     -->
   <xsl:template name="GetRefPrefix">
      <xsl:param name="ref"/>
      <!-- Get namespace prefix -->
      <xsl:value-of select="substring-before($ref, ':')"/>
   </xsl:template>

   <!--
     Returns the namespace of a reference.
     Param(s):
            ref (String) required
                Reference
     -->
   <xsl:template name="GetRefNS">
      <xsl:param name="ref"/>
      <!-- Get namespace prefix -->
      <xsl:variable name="refPrefix" select="substring-before($ref, ':')"/>
      <!-- Get namespace -->
      <xsl:value-of select="/xsd:schema/namespace::*[local-name(.)=normalize-space($refPrefix)]"/>
   </xsl:template>

   <!--
     Returns the declared prefix of this schema's target namespace.
     -->
   <xsl:template name="GetThisPrefix">
      <xsl:if test="/xsd:schema/@targetNamespace">
         <xsl:value-of select="local-name(/xsd:schema/namespace::*[normalize-space(.)=normalize-space(/xsd:schema/@targetNamespace)])"/>
      </xsl:if>
   </xsl:template>

   <!--
     Returns the declared prefix of XML Schema namespace.
     -->
   <xsl:template name="GetXSDPrefix">
      <xsl:value-of select="local-name(/xsd:schema/namespace::*[normalize-space(.)=$XSD_NS])"/>
   </xsl:template>
   <!--
     Prints out a boolean value.
     Param(s):
            boolean (String) required
                Boolean value
     -->
   <xsl:template name="PrintBoolean">
      <xsl:param name="boolean"/>

      <xsl:choose>
         <xsl:when test="normalize-space(translate($boolean,'TRUE', 'true'))='true' or normalize-space($boolean)='1'">
            <xsl:text>yes</xsl:text>
         </xsl:when>
         <xsl:otherwise>
            <xsl:text>no</xsl:text>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Prints out the min/max occurrences of a schema component.
     Param(s):
            component (Node) optional
                Schema component
            minOccurs (String) optional
                Minimum occurrences
            maxOccurs (String) optional
                Maximum occurrences
     -->
   <xsl:template name="PrintOccurs">
      <xsl:param name="component"/>
      <xsl:param name="minOccurs"/>
      <xsl:param name="maxOccurs"/>

      <!-- Get min occurs -->
      <xsl:variable name="min">
         <xsl:choose>
            <xsl:when test="$component and local-name($component)='attribute'">
               <xsl:choose>
                  <xsl:when test="normalize-space($component/@use)='required'">
                     <xsl:text>1</xsl:text>
                  </xsl:when>
                  <xsl:otherwise>
                     <xsl:text>0</xsl:text>
                  </xsl:otherwise>
               </xsl:choose>
            </xsl:when>
            <xsl:otherwise>
               <xsl:choose>
                  <xsl:when test="$component and $component/@minOccurs">
                     <xsl:value-of select="$component/@minOccurs"/>
                  </xsl:when>
                  <xsl:when test="$minOccurs != ''">
                     <xsl:value-of select="$minOccurs"/>
                  </xsl:when>
                  <xsl:otherwise>
                     <xsl:text>1</xsl:text>
                  </xsl:otherwise>
               </xsl:choose>
            </xsl:otherwise>
         </xsl:choose>
      </xsl:variable>
      <!-- Get max occurs -->
      <xsl:variable name="max">
         <xsl:choose>
            <xsl:when test="$component and local-name($component)='attribute'">
               <xsl:choose>
                  <xsl:when test="normalize-space($component/@use)='prohibited'">
                     <xsl:text>0</xsl:text>
                  </xsl:when>
                  <xsl:otherwise>
                     <xsl:text>1</xsl:text>
                  </xsl:otherwise>
               </xsl:choose>
            </xsl:when>
            <xsl:otherwise>
               <xsl:choose>
                  <xsl:when test="($component and normalize-space($component/@maxOccurs)='unbounded') or $maxOccurs='unbounded'">
                     <xsl:text>*</xsl:text>
                  </xsl:when>
                  <xsl:when test="$component and $component/@maxOccurs">
                     <xsl:value-of select="$component/@maxOccurs"/>
                  </xsl:when>
                  <xsl:when test="$maxOccurs != ''">
                     <xsl:value-of select="$maxOccurs"/>
                  </xsl:when>
                  <xsl:otherwise>
                     <xsl:text>1</xsl:text>
                  </xsl:otherwise>
               </xsl:choose>
            </xsl:otherwise>
         </xsl:choose>
      </xsl:variable>

         <xsl:choose>
            <xsl:when test="number($min)=1 and number($max)=1">
               <xsl:text>[1]</xsl:text>
            </xsl:when>
            <xsl:otherwise>
               <xsl:text>[</xsl:text>
               <xsl:value-of select="$min"/>
               <xsl:text>..</xsl:text>
               <xsl:value-of select="$max"/>
               <xsl:text>]</xsl:text>
            </xsl:otherwise>
         </xsl:choose>
   </xsl:template>

   <!--
     Translates occurrence of '#all' in 'block' value
     of element declarations.
     Param(s):
            EBV (String) required
                Value
     -->
   <xsl:template name="PrintBlockSet">
      <xsl:param name="EBV"/>

      <xsl:choose>
         <xsl:when test="normalize-space($EBV)='#all'">
            <xsl:text>restriction, extension, substitution</xsl:text>
         </xsl:when>
         <xsl:otherwise>
            <xsl:value-of select="$EBV"/>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Translates occurrence of '#all' in 'final' value
     of element declarations, and 'block' and 'final' values 
     in complex type definitions.
     Param(s):
            EBV (String) required
                Value
     -->
   <xsl:template name="PrintDerivationSet">
      <xsl:param name="EBV"/>

      <xsl:choose>
         <xsl:when test="normalize-space($EBV)='#all'">
            <xsl:text>restriction, extension</xsl:text>
         </xsl:when>
         <xsl:otherwise>
            <xsl:value-of select="$EBV"/>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Translates occurrence of '#all' in 'final' value
     of simple type definitions.
     Param(s):
            EBV (String) required
                Value
     -->
   <xsl:template name="PrintSimpleDerivationSet">
      <xsl:param name="EBV"/>

      <xsl:choose>
         <xsl:when test="normalize-space($EBV)='#all'">
            <xsl:text>restriction, list, union</xsl:text>
         </xsl:when>
         <xsl:when test="normalize-space($EBV)!=''">
            <xsl:value-of select="$EBV"/>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Print out a URI. If it starts with 'http', a link is provided.
     Param(s):
            uri (String) required
              URI to be printed
     -->
   <xsl:template name="PrintURI">
      <xsl:param name="uri"/>

      <xsl:choose>
         <xsl:when test="starts-with($uri, 'http')">
            <a title="{$uri}" href="{$uri}">
               <xsl:value-of select="$uri"/>
            </a>
         </xsl:when>
         <xsl:otherwise>
            <xsl:value-of select="$uri"/>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Tokenises a whitespace-separated list of values, and
     displays them appropriately.
     Param(s):
            value (String) required
              Whitespace-separated list
            compType (String) optional
              Specify schema component type if values in list are
              schema components, so appropriate links can be provided.
            isInList (boolean) optional
              If true, place each value within 'li' tags.
              Assumes that this template is called within an HTML
              list element.
            separator (String) optional
              Character(s) to use to separate resulting values in list.
              Only used if 'isInList' is false.
              If none is provided, uses a space character, ' '.
            isFirst (boolean) optional
              If false, it's a recursive call from 'PrintWhitespaceList'.
            schemaLoc (String) optional
                Schema file containing this all model group;
                if in current schema, 'schemaLoc' is set to 'this'
     -->
   <xsl:template name="PrintWhitespaceList">
      <xsl:param name="value"/>
      <xsl:param name="compType"/>
      <xsl:param name="isInList">false</xsl:param>
      <xsl:param name="separator"/>
      <xsl:param name="isFirst">true</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>

      <xsl:variable name="token" select="normalize-space(substring-before($value, ' '))"/>
      <xsl:choose>
         <xsl:when test="$token!=''">
            <!-- Whitespace found in value -->
            <!-- Print out token -->
            <xsl:call-template name="PrintWhitespaceListToken">
               <xsl:with-param name="token" select="$token"/>
               <xsl:with-param name="compType" select="$compType"/>
               <xsl:with-param name="isInList" select="$isInList"/>
               <xsl:with-param name="separator" select="$separator"/>
               <xsl:with-param name="isFirst" select="$isFirst"/>
               <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
            </xsl:call-template>
            <!-- Continue parsing -->
            <xsl:variable name="rest" select="normalize-space(substring-after($value, $token))"/>
            <xsl:if test="$rest!=''">
               <xsl:call-template name="PrintWhitespaceList">
                  <xsl:with-param name="value" select="$rest"/>
                  <xsl:with-param name="compType" select="$compType"/>
                  <xsl:with-param name="isInList" select="$isInList"/>
                  <xsl:with-param name="separator" select="$separator"/>
                  <xsl:with-param name="isFirst">false</xsl:with-param>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:call-template>
            </xsl:if>
         </xsl:when>
         <xsl:otherwise>
            <!-- No more whitespaces  -->
            <!-- Print out one last token -->
            <xsl:if test="normalize-space($value)!=''">
               <xsl:call-template name="PrintWhitespaceListToken">
                  <xsl:with-param name="token" select="$value"/>
                  <xsl:with-param name="compType" select="$compType"/>
                  <xsl:with-param name="isInList" select="$isInList"/>
                  <xsl:with-param name="separator" select="$separator"/>
                  <xsl:with-param name="isFirst" select="$isFirst"/>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:call-template>
            </xsl:if>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Helper template for 'PrintWhitespaceList' template,
     which prints out one token in the list.
     Param(s):
            token (String) required
              Token to be printed
            compType (String) optional
              Schema component type of token, if applicable.
            isInList (boolean) optional
              If true, place token within 'li' tags.
            separator (String) optional
              Character(s) use to separate one token from another.
              Only used if 'isInList' is false.
              If none is provided, uses a space character, ' '.
            isFirst (boolean) optional
              If true, token is the first value in the list.
            schemaLoc (String) optional
                Schema file containing this all model group;
                if in current schema, 'schemaLoc' is set to 'this'
     -->
   <xsl:template name="PrintWhitespaceListToken">
      <xsl:param name="token"/>
      <xsl:param name="compType"/>
      <xsl:param name="isInList">false</xsl:param>
      <xsl:param name="separator"/>
      <xsl:param name="isFirst">true</xsl:param>
      <xsl:param name="schemaLoc">this</xsl:param>

      <xsl:variable name="displayValue">
         <xsl:choose>
            <xsl:when test="$compType!=''">
               <xsl:call-template name="PrintCompRef">
                  <xsl:with-param name="ref" select="normalize-space($token)"/>
                  <xsl:with-param name="compType" select="$compType"/>
                  <xsl:with-param name="schemaLoc" select="$schemaLoc"/>
               </xsl:call-template>
            </xsl:when>
            <xsl:otherwise>
               <xsl:value-of select="normalize-space($token)"/>
            </xsl:otherwise>
         </xsl:choose>
      </xsl:variable>

      <xsl:choose>
         <xsl:when test="$isInList!='false'">
            <li>
               <xsl:copy-of select="$displayValue"/>
            </li>
         </xsl:when>
         <xsl:when test="$isFirst!='true'">
            <xsl:choose>
               <xsl:when test="$separator!=''">
                  <xsl:value-of select="$separator"/>
               </xsl:when>
               <xsl:otherwise>
                  <xsl:text> </xsl:text>
               </xsl:otherwise>
            </xsl:choose>
            <xsl:copy-of select="$displayValue"/>
         </xsl:when>
         <xsl:otherwise>
            <xsl:copy-of select="$displayValue"/>
         </xsl:otherwise>
      </xsl:choose>
   </xsl:template>

   <!--
     Print out a wildcard.
     Param(s):
            componentType (attribute|element) required
              XML Schema component type
            namespaces (String) required
              Namespace attribute of wildcard
            processContents (String) required
              Process contents attribute of wildcard
            namespaces (String) required
              Namespace attribute of wildcard
     -->
   <xsl:template name="PrintWildcard">
      <xsl:param name="componentType">element</xsl:param>
      <xsl:param name="namespace"/>
      <xsl:param name="processContents"/>
      <xsl:param name="minOccurs"/>
      <xsl:param name="maxOccurs"/>

      <xsl:text>Allow any </xsl:text>
      <xsl:value-of select="$componentType"/>
      <xsl:text>s from </xsl:text>

      <xsl:choose>
         <!-- ##any -->
         <xsl:when test="not($namespace) or normalize-space($namespace)='##any'">
            <xsl:text>any namespace</xsl:text>
         </xsl:when>
         <!-- ##other -->
         <xsl:when test="normalize-space($namespace)='##other'">
            <xsl:text>a namespace other than this schema's namespace</xsl:text>
         </xsl:when>
         <!-- ##targetNamespace, ##local, specific namespaces -->
         <xsl:otherwise>
            <!-- ##targetNamespace -->
            <xsl:variable name="hasTargetNS">
               <xsl:if test="contains($namespace, '##targetNamespace')">
                  <xsl:text>true</xsl:text>
               </xsl:if>
            </xsl:variable>
            <!-- ##local -->
            <xsl:variable name="hasLocalNS">
               <xsl:if test="contains($namespace, '##local')">
                  <xsl:text>true</xsl:text>
               </xsl:if>
            </xsl:variable>
            <!-- Specific namespaces -->
            <!-- Remove '##targetNamespace' string if any-->
            <xsl:variable name="temp">
               <xsl:choose>
                  <xsl:when test="$hasTargetNS='true'">
                     <xsl:value-of select="concat(substring-before($namespace, '##targetNamespace'), substring-after($namespace, '##targetNamespace'))"/>
                  </xsl:when>
                  <xsl:otherwise>
                     <xsl:value-of select="$namespace"/>
                  </xsl:otherwise>
               </xsl:choose>
            </xsl:variable>
            <!-- Remove '##local' string if any -->
            <xsl:variable name="specificNS">
               <xsl:choose>
                  <xsl:when test="$hasLocalNS='true'">
                     <xsl:value-of select="concat(substring-before($temp, '##local'), substring-after($temp, '##local'))"/>
                  </xsl:when>
                  <xsl:otherwise>
                     <xsl:value-of select="$temp"/>
                  </xsl:otherwise>
               </xsl:choose>
            </xsl:variable>
            <xsl:variable name="hasSpecificNS">
               <xsl:if test="normalize-space($specificNS)!=''">
                  <xsl:text>true</xsl:text>
               </xsl:if>
            </xsl:variable>

            <xsl:if test="$hasLocalNS='true'">
               <xsl:text>no namespace</xsl:text>
            </xsl:if>

            <xsl:if test="$hasTargetNS='true'">
               <xsl:choose>
                  <xsl:when test="$hasLocalNS='true' and $hasSpecificNS!='true'">
                     <xsl:text> and </xsl:text>
                  </xsl:when>
                  <xsl:when test="$hasLocalNS='true'">
                     <xsl:text>, </xsl:text>
                  </xsl:when>
               </xsl:choose>
               <xsl:text>this schema's namespace</xsl:text>
            </xsl:if>

            <xsl:if test="$hasSpecificNS='true'">
               <xsl:choose>
                  <xsl:when test="$hasTargetNS='true' and $hasLocalNS='true'">
                     <xsl:text>, and </xsl:text>
                  </xsl:when>
                  <xsl:when test="$hasTargetNS='true' or $hasLocalNS='true'">
                     <xsl:text> and </xsl:text>
                  </xsl:when>
               </xsl:choose>
               <xsl:text>the following namespace(s): </xsl:text>
               <xsl:call-template name="PrintWhitespaceList">
                  <xsl:with-param name="value" select="normalize-space($specificNS)"/>
                  <xsl:with-param name="separator">,</xsl:with-param>
               </xsl:call-template>
            </xsl:if>
         </xsl:otherwise>
      </xsl:choose>
      <!-- Process contents -->
      <xsl:text> (</xsl:text>
      <xsl:choose>
         <xsl:when test="$processContents">
            <xsl:value-of select="normalize-space($processContents)"/>
         </xsl:when>
         <xsl:otherwise>
            <xsl:text>strict</xsl:text>
         </xsl:otherwise>
      </xsl:choose>
      <xsl:text> validation)</xsl:text>
      <xsl:text>.</xsl:text>

      <!-- Print min/max occurs -->
      <xsl:if test="$componentType='element'">
         <xsl:text> </xsl:text>
         <xsl:call-template name="PrintOccurs">
            <xsl:with-param name="minOccurs" select="$minOccurs"/>
            <xsl:with-param name="maxOccurs" select="$maxOccurs"/>
         </xsl:call-template>
      </xsl:if>
   </xsl:template>

   <!--
     Print out the pattern property for derivations by
     restriction on simple content.
     Param(s):
            simpleRestrict (Node) required
              'restriction' element
     -->
   <xsl:template name="PrintPatternFacet">
      <xsl:param name="simpleRestrict"/>

      <xsl:if test="$simpleRestrict/xsd:pattern">
        <xsl:text>pattern</xsl:text>
         <xsl:text> = </xsl:text>
         <xsl:value-of select="$simpleRestrict/xsd:pattern/@value"/>
      </xsl:if>
   </xsl:template>

   <!--
     Print out the total digits property for derivations by
     restriction on simple content.
     Param(s):
            simpleRestrict (Node) required
              'restriction' element
     -->
   <xsl:template name="PrintTotalDigitsFacet">
      <xsl:param name="simpleRestrict"/>

      <xsl:if test="$simpleRestrict/xsd:totalDigits">
        <xsl:text>total no. of digits</xsl:text>
         <xsl:text> = </xsl:text>
         <xsl:value-of select="$simpleRestrict/xsd:totalDigits/@value"/>
      </xsl:if>
   </xsl:template>

   <!--
     Print out the fraction digits property for derivations by
     restriction on simple content.
     Param(s):
            simpleRestrict (Node) required
              'restriction' element
     -->
   <xsl:template name="PrintFractionDigitsFacet">
      <xsl:param name="simpleRestrict"/>

      <xsl:if test="$simpleRestrict/xsd:fractionDigits">
         <xsl:text>no. of fraction digits</xsl:text>
         <xsl:text> = </xsl:text>
         <xsl:value-of select="$simpleRestrict/xsd:fractionDigits/@value"/>
      </xsl:if>
   </xsl:template>

   <!--
     Print out the enumeration list for derivations by
     restriction on simple content.
     Param(s):
            simpleRestrict (Node) required
              'restriction' element
     -->
   <xsl:template name="PrintEnumFacets">
      <xsl:param name="simpleRestrict"/>

      <xsl:if test="$simpleRestrict/xsd:enumeration">
        <xsl:text>value</xsl:text>
         <xsl:text> comes from list: {</xsl:text>

         <xsl:for-each select="$simpleRestrict/xsd:enumeration">
            <xsl:if test="position()!=1">
               <xsl:text>|</xsl:text>
            </xsl:if>
            <xsl:text>'</xsl:text>
            <xsl:value-of select="@value"/>
            <xsl:text>'</xsl:text>
         </xsl:for-each>

         <xsl:text>}</xsl:text>
      </xsl:if>
   </xsl:template>

   <!--
     Print out the length property for derivations by
     restriction on simple content.
     Param(s):
            simpleRestrict (Node) required
              'restriction' element
     -->
   <xsl:template name="PrintLengthFacets">
      <xsl:param name="simpleRestrict"/>

      <xsl:choose>
         <xsl:when test="$simpleRestrict/xsd:length">
            <xsl:text>length</xsl:text>
            <xsl:text> = </xsl:text>
            <xsl:value-of select="$simpleRestrict/xsd:length/@value"/>
         </xsl:when>
         <xsl:when test="$simpleRestrict/xsd:minLength">
            <xsl:text>length</xsl:text>
            <xsl:text> >= </xsl:text>
            <xsl:value-of select="$simpleRestrict/xsd:minLength/@value"/>
         </xsl:when>
         <xsl:when test="$simpleRestrict/xsd:maxLength">
           <xsl:text>length</xsl:text>
            <xsl:text> &lt;= </xsl:text>
            <xsl:value-of select="$simpleRestrict/xsd:maxLength/@value"/>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Print out the whitespace property for derivations by
     restriction on simple content.
     Param(s):
            simpleRestrict (Node) required
              'restriction' element
     -->
   <xsl:template name="PrintWhitespaceFacet">
      <xsl:param name="simpleRestrict"/>

      <xsl:variable name="facetValue" select="normalize-space(translate($simpleRestrict/xsd:whiteSpace/@value, 'ACELOPRSV', 'aceloprsv'))"/>

      <xsl:choose>
         <xsl:when test="$facetValue='preserve'">
           <xsl:text>Whitespace policy: </xsl:text>
            <xsl:call-template name="PrintGlossaryTermRef">
               <xsl:with-param name="code">PreserveWS</xsl:with-param>
               <xsl:with-param name="term">preserve</xsl:with-param>
            </xsl:call-template>
         </xsl:when>
         <xsl:when test="$facetValue='replace'">
           <xsl:text>Whitespace policy: </xsl:text>
            <xsl:call-template name="PrintGlossaryTermRef">
               <xsl:with-param name="code">ReplaceWS</xsl:with-param>
               <xsl:with-param name="term">replace</xsl:with-param>
            </xsl:call-template>
         </xsl:when>
         <xsl:when test="$facetValue='collapse'">
            <xsl:text>Whitespace policy: </xsl:text>
            <xsl:call-template name="PrintGlossaryTermRef">
               <xsl:with-param name="code">CollapseWS</xsl:with-param>
               <xsl:with-param name="term">collapse</xsl:with-param>
            </xsl:call-template>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Print out the value ranges for derivations by
     restriction on simple content.
     Param(s):
            simpleRestrict (Node) required
              'restriction' element
     -->
   <xsl:template name="PrintRangeFacets">
      <xsl:param name="simpleRestrict"/>

      <xsl:choose>
         <xsl:when test="($simpleRestrict/xsd:minInclusive or $simpleRestrict/xsd:minExclusive) and ($simpleRestrict/xsd:maxInclusive or $simpleRestrict/xsd:maxExclusive)">
            <xsl:choose>
               <xsl:when test="$simpleRestrict/xsd:minInclusive">
                  <xsl:value-of select="$simpleRestrict/xsd:minInclusive/@value"/>
                  <xsl:text> &lt;= </xsl:text>
               </xsl:when>
               <xsl:otherwise>
                  <xsl:value-of select="$simpleRestrict/xsd:minExclusive/@value"/>
                  <xsl:text> &lt; </xsl:text>
               </xsl:otherwise>
            </xsl:choose>
               <xsl:text>value</xsl:text>
            <xsl:choose>
               <xsl:when test="$simpleRestrict/xsd:maxInclusive">
                  <xsl:text> &lt;= </xsl:text>
                  <xsl:value-of select="$simpleRestrict/xsd:maxInclusive/@value"/>
               </xsl:when>
               <xsl:otherwise>
                  <xsl:text> &lt; </xsl:text>
                  <xsl:value-of select="$simpleRestrict/xsd:maxExclusive/@value"/>
               </xsl:otherwise>
            </xsl:choose>
         </xsl:when>
         <xsl:when test="$simpleRestrict/xsd:minInclusive">
               <xsl:text>value</xsl:text>
            <xsl:text> >= </xsl:text>
            <xsl:value-of select="$simpleRestrict/xsd:minInclusive/@value"/>
         </xsl:when>
         <xsl:when test="$simpleRestrict/xsd:minExclusive">
               <xsl:text>value</xsl:text>
            <xsl:text> > </xsl:text>
            <xsl:value-of select="$simpleRestrict/xsd:minExclusive/@value"/>
         </xsl:when>
         <xsl:when test="$simpleRestrict/xsd:maxInclusive">
               <xsl:text>value</xsl:text>
            <xsl:text> &lt;= </xsl:text>
            <xsl:value-of select="$simpleRestrict/xsd:maxInclusive/@value"/>
         </xsl:when>
         <xsl:when test="$simpleRestrict/xsd:maxExclusive">
               <xsl:text>value</xsl:text>
            <xsl:text> &lt; </xsl:text>
            <xsl:value-of select="$simpleRestrict/xsd:maxExclusive/@value"/>
         </xsl:when>
      </xsl:choose>
   </xsl:template>

   <!--
     Translates occurrences of a string
     in a piece of text with another string.
     Param(s):
            value (String) required
                Text to translate
            strToReplace (String) required
                String to be replaced
            replacementStr (String) required
                Replacement text
     -->
   <xsl:template name="TranslateStr">
      <xsl:param name="value"/>
      <xsl:param name="strToReplace"/>
      <xsl:param name="replacementStr"/>

      <xsl:if test="$value != ''">
         <xsl:variable name="beforeText" select="substring-before($value, $strToReplace)"/>
         <xsl:choose>
            <xsl:when test="$beforeText != ''">
               <xsl:value-of select="$beforeText"/>
               <xsl:value-of select="$replacementStr"/>
               <xsl:call-template name="TranslateStr">
                  <xsl:with-param name="value" select="substring-after($value, $strToReplace)"/>
                  <xsl:with-param name="strToReplace" select="$strToReplace"/>
                  <xsl:with-param name="replacementStr" select="$replacementStr"/>
               </xsl:call-template>
            </xsl:when>
            <xsl:otherwise>
               <xsl:value-of select="$value"/>
            </xsl:otherwise>
         </xsl:choose>
      </xsl:if>
   </xsl:template>

   <xsl:template name="HandleError">
      <xsl:param name="errorMsg"/>
      <xsl:param name="isTerminating">false</xsl:param>

      <xsl:text>ERROR: </xsl:text>
      <xsl:value-of select="$errorMsg"/>
   </xsl:template>
</xsl:stylesheet>
