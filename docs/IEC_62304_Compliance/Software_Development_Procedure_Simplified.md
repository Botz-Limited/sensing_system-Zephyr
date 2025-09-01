# Software Development Procedure (Simplified)

**Version:** 2.0  
**Date:** July 2025  
**Status:** Simplified and Improved  
**Classification:** IEC 62304 Compliant

---

## 1. Purpose

This procedure defines how Botz Innovation develops, verifies, and releases software for medical devices in compliance with IEC 62304:2006.

**Key Goals:**
- Ensure software safety and quality
- Meet regulatory requirements
- Provide clear, actionable steps for developers

---

## 2. Scope

This procedure applies to:
- ✅ Medical device software (standalone or embedded)
- ✅ Software that controls or monitors medical devices
- ✅ Data management software for medical devices
- ✅ All Botz Innovation personnel and approved partners developing such software

**Note:** Approved partners may use their own procedures if documented in contracts.

---

## 3. Quick Reference - Roles

| Role | Abbreviation | Responsibility |
|------|--------------|----------------|
| **Lead Software Developer** | LSD | Technical leadership, risk input, planning |
| **Software Quality Engineer** | SQE | Quality assurance, verification oversight |
| **Project Manager** | PM | Schedule, resources, coordination |
| **Product Manager** | PrM | Requirements, customer interface |
| **Design Manager** | DM | Design decisions, architecture |

---

## 4. Software Classification (Critical!)

### 4.1 IEC 62304 Safety Classes

| Class | Definition | Example |
|-------|------------|---------|
| **A** | No injury possible | Data display, logging |
| **B** | Non-serious injury possible | Monitoring with alarms |
| **C** | Death or serious injury possible | Therapy delivery, life support |

### 4.2 FDA Level of Concern

| Level | Definition | Documentation Required |
|-------|------------|----------------------|
| **Minor** | Unlikely to cause injury | Basic |
| **Moderate** | Could cause minor injury | Standard |
| **Major** | Could cause death/serious injury | Comprehensive |

**Action:** Classify EVERY software module at the start of development!

---

## 5. Development Process Overview

### 5.1 Integration with General Project Gates

```mermaid
gantt
    title Software Development in General Project Timeline
    dateFormat  YYYY-MM-DD
    section Project Gates
    DR-0 Concept          :milestone, dr0, 2025-01-01, 0d
    DR-1 Requirements     :milestone, dr1, 2025-02-01, 0d
    DR-2 Design Freeze    :milestone, dr2, 2025-06-01, 0d
    DR-3 V&V Complete     :milestone, dr3, 2025-09-01, 0d
    DR-4 Release          :milestone, dr4, 2025-10-01, 0d
    
    section Software Dev
    Planning              :active, plan, after dr1, 2w
    Requirements          :req, after plan, 3w
    Architecture          :arch, after req, 2w
    Sprint 1-3 (Dev)      :dev1, after arch, 6w
    Sprint 4-6 (Test)     :test1, after dev1, 6w
    Integration           :int, after test1, 2w
    Release Prep          :rel, after int, 2w
```

### 5.2 Software Development Mapping to Project Gates

```mermaid
flowchart TB
    subgraph "General Project Gates"
        DR0[DR-0: Concept Approval]
        DR1[DR-1: Requirements Freeze]
        DR2[DR-2: Design Freeze]
        DR3[DR-3: V&V Complete]
        DR4[DR-4: Product Release]
    end
    
    subgraph "Software Deliverables"
        SWDP[Software Dev Plan]
        SWRS[Software Requirements]
        SWDS[Software Design]
        CODE[Implemented Software]
        SWVR[Verification Report]
        SWRE[Release Package]
    end
    
    DR0 --> DR1
    DR1 --> SWDP
    DR1 --> SWRS
    SWRS --> DR2
    SWDS --> DR2
    DR2 --> CODE
    CODE --> SWVR
    SWVR --> DR3
    DR3 --> SWRE
    SWRE --> DR4
    
    style DR1 fill:#f9f,stroke:#333,stroke-width:4px
    style DR2 fill:#f9f,stroke:#333,stroke-width:4px
    style DR3 fill:#f9f,stroke:#333,stroke-width:4px
    style DR4 fill:#f9f,stroke:#333,stroke-width:4px

```

### 5.3 Agile Development with IEC 62304 Compliance

```mermaid
flowchart LR
    subgraph "Sprint Planning"
        PB[Product Backlog]
        SP[Sprint Planning]
        SB[Sprint Backlog]
    end
    
    subgraph "Sprint Execution (2-4 weeks)"
        daily[Daily Standup]
        dev[Development]
        test[Testing]
        review[Code Review]
    end
    
    subgraph "Sprint Closure"
        demo[Sprint Demo]
        retro[Retrospective]
        doc[Update Docs]
    end
    
    subgraph "IEC 62304 Artifacts"
        SWRS_U[SWRS Updates]
        SWDS_U[SWDS Updates]
        TEST[Test Records]
        TRACE[Traceability]
    end
    
    PB --> SP
    SP --> SB
    SB --> daily
    daily --> dev
    dev --> test
    test --> review
    review --> demo
    demo --> retro
    retro --> doc
    
    dev -.-> SWDS_U
    test -.-> TEST
    doc -.-> SWRS_U
    doc -.-> TRACE
    
    style daily fill:#bbf,stroke:#333
    style dev fill:#bbf,stroke:#333
    style test fill:#bbf,stroke:#333
    style review fill:#bbf,stroke:#333
```

### 5.4 Agile Sprint Mapping to IEC 62304

| Sprint Phase | IEC 62304 Activity | Deliverables |
|--------------|-------------------|--------------|
| **Sprint Planning** | Requirements Analysis | Updated SWRS items |
| **Design Sessions** | Software Design | SWDS for sprint items |
| **Development** | Implementation | Code + Unit Tests |
| **Testing** | Verification | Test Records |
| **Sprint Review** | Design Review | Review Meeting Minutes |
| **Documentation** | Traceability Update | Updated trace matrix |

### 5.5 Key Principles for Agile Medical Device Development

1. **Documentation in Parallel** - Not at the end
2. **Incremental Compliance** - Build compliance artifacts sprint by sprint
3. **Definition of Done** includes:
   - Code complete and reviewed
   - Tests written and passed
   - Documentation updated
   - Traceability maintained
4. **Regular Reviews** - Each sprint end = mini design review
5. **Continuous Integration** - Automated testing for regression

### 5.6 V-Model with Agile Sprints

```mermaid
flowchart TB
    subgraph "Left Side - Specification"
        CR[Customer Requirements<br/>DR-1]
        PR[Product Requirements]
        SWRS[Software Requirements<br/>Specification]
        SWDS_A[SW Architecture<br/>Design]
        SWDS_D[SW Detailed<br/>Design]
        IMPL[Implementation]
    end
    
    subgraph "Right Side - Verification"
        VAL[Validation<br/>DR-4]
        PVT[Product Verification<br/>& Testing]
        SWRT[SW Requirements<br/>Testing DR-3]
        SWDT[SW Design<br/>Testing]
        UNIT[Unit<br/>Testing]
    end
    
    subgraph "Agile Sprints"
        S1[Sprint 1<br/>Foundation]
        S2[Sprint 2<br/>Core Features]
        S3[Sprint 3<br/>Advanced]
        S4[Sprint 4<br/>Integration]
        S5[Sprint 5<br/>Hardening]
        S6[Sprint 6<br/>Release]
    end
    
    CR --> PR
    PR --> SWRS
    SWRS --> SWDS_A
    SWDS_A --> SWDS_D
    SWDS_D --> IMPL
    
    IMPL --> UNIT
    UNIT --> SWDT
    SWDT --> SWRT
    SWRT --> PVT
    PVT --> VAL
    
    CR -.->|traces to| VAL
    PR -.->|traces to| PVT
    SWRS -.->|traces to| SWRT
    SWDS_A -.->|traces to| SWDT
    SWDS_D -.->|traces to| UNIT
    
    S1 --> S2
    S2 --> S3
    S3 --> S4
    S4 --> S5
    S5 --> S6
    
    S1 -.-> SWRS
    S2 -.-> SWDS_A
    S3 -.-> IMPL
    S4 -.-> SWDT
    S5 -.-> SWRT
    S6 -.-> VAL
    
    style CR fill:#f9f,stroke:#333,stroke-width:2px
    style VAL fill:#f9f,stroke:#333,stroke-width:2px
    style S1 fill:#bbf,stroke:#333
    style S2 fill:#bbf,stroke:#333
    style S3 fill:#bbf,stroke:#333
    style S4 fill:#bbf,stroke:#333
    style S5 fill:#bbf,stroke:#333
    style S6 fill:#bbf,stroke:#333
```

### 5.7 Sprint-Based Deliverables Timeline

```mermaid
gantt
    title Agile Sprint Deliverables for IEC 62304
    dateFormat  YYYY-MM-DD
    
    section Planning
    SWDP & SWVP           :done, swdp, 2025-02-01, 14d
    Initial SWRS          :done, swrs1, after swdp, 14d
    
    section Sprint 1-2
    Core Requirements     :active, req1, 2025-03-01, 28d
    Architecture Design   :arch, after req1, 14d
    Basic Implementation  :impl1, 2025-03-15, 28d
    
    section Sprint 3-4  
    Feature Development   :feat, 2025-04-15, 28d
    Integration Testing   :int, 2025-05-01, 21d
    Design Verification   :dv, after int, 14d
    
    section Sprint 5-6
    System Testing        :sys, 2025-06-01, 21d
    Bug Fixes            :fix, 2025-06-15, 14d
    Documentation        :doc, 2025-06-20, 14d
    Release Prep         :rel, 2025-07-01, 7d
    
    section Reviews
    SWDR-0               :milestone, sdr0, 2025-03-01, 0d
    SWDR-1               :milestone, sdr1, 2025-04-01, 0d
    SWDR-2               :milestone, sdr2, 2025-05-01, 0d
    SWDR-3               :milestone, sdr3, 2025-06-01, 0d
    SWDR-4               :milestone, sdr4, 2025-07-01, 0d
```

---

## 6. Agile Practices for Medical Device Software

### 6.1 Sprint Structure for IEC 62304

```mermaid
flowchart TD
    subgraph "2-Week Sprint"
        Day1[Day 1: Sprint Planning]
        Day2[Days 2-8: Development]
        Day9[Day 9: Code Freeze]
        Day10[Day 10: Sprint Review]
    end
    
    subgraph "Continuous Activities"
        Daily[Daily Standup]
        CI[Continuous Integration]
        Doc[Documentation Updates]
    end
    
    subgraph "Sprint Outputs"
        Code[Tested Code]
        Docs[Updated SWRS/SWDS]
        Tests[Test Records]
        Trace[Traceability Matrix]
    end
    
    Day1 --> Day2
    Day2 --> Day9
    Day9 --> Day10
    
    Day2 --> Daily
    Day2 --> CI
    Day2 --> Doc
    
    Day10 --> Code
    Day10 --> Docs
    Day10 --> Tests
    Day10 --> Trace
```

### 6.2 Definition of Done for Medical Device Software

| Level | Criteria | Class A | Class B/C |
|-------|----------|---------|-----------|
| **Code Complete** | | | |
| - Implements design | ��� | ✅ |
| - Follows coding standards | ✅ | ✅ |
| - Compiles without warnings | ✅ | ✅ |
| **Testing Complete** | | | |
| - Unit tests pass | Optional | ✅ |
| - Integration tests pass | ❌ | ✅ |
| - Code coverage >80% | ❌ | ✅ |
| **Review Complete** | | | |
| - Code reviewed | ✅ | ✅ |
| - Static analysis clean | Optional | ✅ |
| **Documentation Complete** | | | |
| - Design updated (SWDS) | ✅ | ✅ |
| - Requirements traced | ✅ | ✅ |
| - Test records filed | ✅ | ✅ |

### 6.3 Agile Artifacts Mapping

| Agile Artifact | IEC 62304 Equivalent | When Updated |
|----------------|---------------------|--------------|
| Product Backlog | SWRS (partial) | Sprint Planning |
| Sprint Backlog | SWDS items | Sprint Planning |
| User Stories | SWRS items | Refinement |
| Acceptance Criteria | Test Cases | Story Creation |
| Sprint Review Notes | Design Review Minutes | Sprint End |
| Burndown Chart | Progress Tracking | Daily |
| Definition of Done | Acceptance Criteria | Planning |

### 6.4 Risk-Based Sprint Planning

```mermaid
flowchart LR
    subgraph "Backlog Prioritization"
        High[High Risk<br/>Class C]
        Med[Medium Risk<br/>Class B]
        Low[Low Risk<br/>Class A]
    end
    
    subgraph "Sprint Allocation"
        S1[Sprint 1-2<br/>High Risk Items]
        S2[Sprint 3-4<br/>Medium Risk]
        S3[Sprint 5-6<br/>Low Risk]
    end
    
    subgraph "Testing Focus"
        T1[Extensive Testing<br/>100% Coverage]
        T2[Standard Testing<br/>80% Coverage]
        T3[Basic Testing<br/>Key Paths]
    end
    
    High --> S1
    Med --> S2
    Low --> S3
    
    S1 --> T1
    S2 --> T2
    S3 --> T3
    
    style High fill:#f99,stroke:#333
    style Med fill:#ff9,stroke:#333
    style Low fill:#9f9,stroke:#333
```

---

## 7. Phase 1: Planning

### 7.1 Create Software Development Plan (SDP)

**Required Elements:**

| Element | Class A | Class B/C | Template |
|---------|---------|-----------|----------|
| Team & Roles | ✅ | ✅ | In D&D Plan |
| Schedule | ✅ | ✅ | In D&D Plan |
| Verification Plan | ✅ | ✅ | SWVP Template |
| Configuration Mgmt | ✅ | ✅ | SWCP Template |
| Tools List | ✅ | ✅ + Qualification | Tools Table |
| Integration Plan | ❌ | ✅ | In D&D Plan |

### 7.2 Key Planning Decisions

1. **Define Coding Standards** (e.g., MISRA-C, Google Style)
2. **Select Development Tools** 
   - Class A: Any tools OK
   - Class B/C: Must be qualified per WI-XXXX
3. **Set Up Version Control** (Git, SVN, etc.)
4. **Identify SOUP/OTS Software** early

---

## 7. Phase 2: Requirements & Design

### 7.1 Software Requirements (SWRS)

**Process:**
```
Customer Requirements → Product Requirements → Software Requirements
                    ↓                      ↓
                 (if applicable)      Must include:
                                     - Functional requirements
                                     - Performance requirements  
                                     - Interface requirements
                                     - Safety requirements from risk analysis
```

**Requirements Checklist:**
- [ ] Each requirement has unique ID
- [ ] Each requirement is testable
- [ ] Safety class assigned per module
- [ ] No contradictions
- [ ] Traceable to source

### 7.2 Software Design (SWDS)

**Two Levels Required:**

1. **Architecture Design**
   - High-level structure
   - Module interfaces
   - SOUP integration
   
2. **Detailed Design**
   - Implementation details
   - Algorithms
   - Data structures
   - No ad-hoc decisions during coding!

**Design Review Checklist:**
- [ ] Implements all requirements
- [ ] Modules properly separated
- [ ] Testable design
- [ ] SOUP items identified and qualified

---

## 8. Phase 3: Implementation & Unit Testing

### 8.1 Implementation

**Rules:**
1. Follow coding standards defined in SWVP
2. Implement per detailed design (no ad-hoc changes)
3. Document any deviations

### 8.2 Unit Verification

| Activity | Class A | Class B/C | Output |
|----------|---------|-----------|--------|
| Code Review | Recommended | Required | CRR |
| Static Analysis | Recommended | Required | CRR |
| Unit Tests | Recommended | Required | Test Records |
| Compiler Warnings | Check | Zero tolerance | CRR |

**Acceptance Criteria:**
- Coding standards met
- No compiler errors
- Static analysis passed
- Unit tests passed
- Deviations documented with rationale

---

## 9. Phase 4: Integration & System Testing

### 9.1 Integration Testing (Class B/C only)

**Process:**
1. Integrate units per integration plan
2. Test interfaces between units
3. Verify no unexpected interactions
4. Document results

### 9.2 Design Verification (SWDS Testing)

**Purpose:** Verify implementation matches design

**Methods:**
- Manual testing
- Automated testing
- Review test coverage

### 9.3 Requirements Verification (SWRS Testing)

**Purpose:** Verify software meets requirements

**Key Points:**
- Test complete integrated software
- Include SOUP components
- Test risk control measures
- Document all failures in SWFR

---

## 10. Phase 5: Release

### 10.1 Release Checklist

**Documentation Review:**
- [ ] All requirements verified
- [ ] All design elements tested
- [ ] Code reviews complete
- [ ] Test records complete
- [ ] Failures resolved or documented in USWAR

**Technical Review:**
- [ ] Source code archived
- [ ] Build environment documented
- [ ] Version numbers assigned
- [ ] Release notes created

### 10.2 Unresolved Anomalies (USWAR)

**For each unresolved issue:**
1. Document the anomaly
2. Assess risk impact
3. Justify why not fixed
4. Get approval from PM and QA

---

## 11. Key Documents Summary

### Required for All Classes:

| Document | Purpose | When Created |
|----------|---------|--------------|
| SWRS | Software Requirements | Planning/Requirements |
| SWDS | Software Design | Design Phase |
| SWVP | Verification Plan | Planning |
| SWCP | Configuration Plan | Planning |
| Test Records | Evidence of testing | Testing |
| SWVR | Verification Summary | Release |
| SWRE | Release Documentation | Release |

### Additional for Class B/C:

| Document | Purpose |
|----------|---------|
| Integration Plan | How to integrate units |
| CRR | Code Review Reports |
| Tool Qualification | Evidence tools are suitable |

---

## 12. Common Pitfalls to Avoid

1. **Starting coding before design is complete**
   - Solution: Complete and review SWDS first

2. **Inadequate SOUP documentation**
   - Solution: List all SOUP with versions early

3. **Missing traceability**
   - Solution: Use requirement IDs consistently

4. **Informal testing**
   - Solution: Document all tests, even negative results

5. **Ignoring compiler warnings**
   - Solution: Justify each accepted warning

---

## 13. Quick Decision Trees

### 13.1 Do I need this document?

```
Is it medical software?
├─ No → Follow company standards only
└─ Yes → What class?
    ├─ Class A → Basic documentation set
    ├─ Class B → Full documentation + integration
    └─ Class C → Full documentation + extra rigor
```

### 13.2 What to do with a test failure?

```
Test Failed
├─ Is it a real failure?
│   ├─ No → Fix test, document
│   └─ Yes → What type?
│       ├─ Requirement wrong → Update SWRS, retest all
│       ├─ Design wrong → Update SWDS, reimplement
│       ├─ Code wrong → Fix code, retest unit
│       └─ Test wrong → Fix test, rerun
└─ Document in SWFR
```

---

## 14. Templates and Tools

### Available Templates:
- Software Development Plan (in D&D Plan)
- Software Verification Plan (SWVP)
- Software Configuration Plan (SWCP)
- Software Requirements Specification (SWRS)
- Software Design Specification (SWDS)
- Test Record Templates
- Software Failure Report (SWFR)
- Code Review Report (CRR)

### Recommended Tools:
- **Version Control**: Git, SVN
- **Static Analysis**: PC-Lint, Coverity, PVS-Studio
- **Unit Testing**: Unity, CppUTest, Google Test
- **Documentation**: Markdown, Doxygen
- **Requirements Management**: DOORS, Jira, Excel (with discipline)

---

## 15. Gate Deliverables Mapping

### 15.1 Software Deliverables by Project Gate

```mermaid
flowchart TD
    subgraph "DR-1: Requirements Freeze"
        DR1_IN[Inputs:<br/>- Customer Requirements<br/>- Feasibility Study<br/>- Risk Analysis]
        DR1_OUT[SW Outputs:<br/>- SW Development Plan<br/>- Initial SWRS<br/>- SW Risk Classification]
    end
    
    subgraph "DR-2: Design Freeze"
        DR2_IN[Inputs:<br/>- Approved SWRS<br/>- Architecture Decisions]
        DR2_OUT[SW Outputs:<br/>- Complete SWDS<br/>- Test Plans<br/>- Updated SWRS]
    end
    
    subgraph "DR-3: V&V Complete"
        DR3_IN[Inputs:<br/>- Implemented Software<br/>- Test Results]
        DR3_OUT[SW Outputs:<br/>- SWVR<br/>- Test Records<br/>- USWAR]
    end
    
    subgraph "DR-4: Release"
        DR4_IN[Inputs:<br/>- Verified Software<br/>- All Documentation]
        DR4_OUT[SW Outputs:<br/>- SWRE<br/>- Source Code Archive<br/>- Final Documentation]
    end
    
    DR1_OUT --> DR2_IN
    DR2_OUT --> DR3_IN
    DR3_OUT --> DR4_IN
```

### 15.2 Gate Checklist Details

#### DR-1: Requirements Freeze
**Software Deliverables Required:**
- [ ] Software Development Plan (SWDP) - integrated in D&D Plan
- [ ] Software Verification Plan (SWVP)
- [ ] Software Configuration Management Plan (SWCP)
- [ ] Initial Software Requirements (SWRS) - at least high-level
- [ ] Software safety classification documented
- [ ] Development team identified with roles

#### DR-2: Design Freeze  
**Software Deliverables Required:**
- [ ] Complete SWRS - all requirements defined
- [ ] Software Architecture Design (SWDS)
- [ ] Software Detailed Design (SWDS)
- [ ] Test specifications drafted
- [ ] SOUP items identified and listed
- [ ] Integration plan (for Class B/C)
- [ ] Traceability matrix started

#### DR-3: V&V Complete
**Software Deliverables Required:**
- [ ] All code implemented and reviewed
- [ ] Unit test results (Class B/C)
- [ ] Integration test results (Class B/C)
- [ ] Software verification test records
- [ ] Software Verification Report (SWVR)
- [ ] Unresolved anomalies documented (USWAR)
- [ ] Updated traceability matrix

#### DR-4: Product Release
**Software Deliverables Required:**
- [ ] Software Release document (SWRE)
- [ ] Final SWVR with all results
- [ ] Source code archived with build instructions
- [ ] All SOUP items documented with versions
- [ ] Final traceability matrix
- [ ] Residual anomalies reviewed and accepted

### 15.3 Sprint to Gate Alignment

```mermaid
gantt
    title Sprint Deliverables Aligned to Gates
    dateFormat YYYY-MM-DD
    
    section Gates
    DR-1                 :milestone, dr1, 2025-02-15, 0d
    DR-2                 :milestone, dr2, 2025-05-15, 0d
    DR-3                 :milestone, dr3, 2025-08-15, 0d
    DR-4                 :milestone, dr4, 2025-09-30, 0d
    
    section Sprints
    Sprint 0 (Planning)  :s0, 2025-02-01, 14d
    Sprint 1-2 (Req/Des) :s12, after s0, 28d
    Sprint 3-4 (Dev)     :s34, after s12, 28d
    Sprint 5-6 (Test)    :s56, after s34, 28d
    Sprint 7-8 (Hard)    :s78, after s56, 28d
    Sprint 9 (Release)   :s9, after s78, 14d
    
    section Deliverables
    SWDP/SWVP           :del1, 2025-02-01, 14d
    SWRS Complete       :del2, 2025-03-01, 28d
    SWDS Complete       :del3, 2025-04-01, 28d
    Code Complete       :del4, 2025-05-15, 28d
    Testing Complete    :del5, 2025-07-01, 28d
    Documentation       :del6, 2025-08-15, 28d
```

---

## 16. Regulatory Submission Requirements

### For FDA Submission:

| Level of Concern | Required Documents |
|-----------------|-------------------|
| Minor | SRS, Architecture, V&V Summary |
| Moderate | Above + Detailed Design, Test Protocols |
| Major | Above + Complete V&V, Source Code Summary |

### For CE Marking:
- Technical File must include software documentation
- Demonstrate IEC 62304 compliance
- Include risk management per ISO 14971

---

## Appendix A: Acronyms

| Acronym | Meaning |
|---------|---------|
| CRR | Code Review Report |
| D&D | Design and Development |
| DDQP | Design, Development and Quality Plan |
| LSD | Lead Software Developer |
| SOUP | Software of Unknown Provenance |
| SQE | Software Quality Engineer |
| SWCP | Software Configuration Plan |
| SWDS | Software Design Specification |
| SWFR | Software Failure Report |
| SWRE | Software Release |
| SWRS | Software Requirements Specification |
| SWVP | Software Verification Plan |
| SWVR | Software Verification Report |
| USWAR | Unresolved Software Anomalies Report |

---

## Appendix B: Agile User Story Template

### User Story Format for Medical Device Software
```
As a [user type]
I want [functionality]
So that [benefit/value]

Acceptance Criteria:
- [ ] Functional requirement met
- [ ] Performance requirement met  
- [ ] Safety requirement met
- [ ] Tested and documented

Regulatory Considerations:
- Safety Class: [A/B/C]
- Risk Controls: [if any]
- Traceability: [SWRS-XXX]
```

### Example User Story
```
As a clinician
I want to see real-time gait metrics
So that I can assess patient progress

Acceptance Criteria:
- [ ] Displays cadence within 2 seconds
- [ ] Updates at least 1Hz
- [ ] Shows data for both feet
- [ ] Handles sensor disconnection gracefully

Regulatory Considerations:
- Safety Class: B (incorrect data could affect treatment)
- Risk Controls: Data validation, range checking
- Traceability: SWRS-047, SWRS-048
```

---

## Appendix C: Sprint Review Checklist

### End of Sprint Review for Medical Device Software

**Code Review**
- [ ] All code peer reviewed
- [ ] Coding standards verified
- [ ] Static analysis completed (Class B/C)
- [ ] Review comments addressed

**Testing**
- [ ] Unit tests written and passing
- [ ] Integration tests completed (Class B/C)
- [ ] Test coverage meets target
- [ ] Test records archived

**Documentation**
- [ ] SWRS updated with new requirements
- [ ] SWDS updated with design changes
- [ ] Test cases documented
- [ ] Traceability matrix current

**Compliance**
- [ ] Safety classification reviewed
- [ ] Risk analysis updated if needed
- [ ] SOUP items documented
- [ ] Change control followed

**Sprint Metrics**
- [ ] Velocity: ___ story points
- [ ] Defects found: ___
- [ ] Defects resolved: ___
- [ ] Technical debt items: ___

---

## Appendix D: Change History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0 | Original | Initial procedure | Botz Team |
| 2.0 | July 2025 | Simplified, added quick references, decision trees | System |
| 2.1 | July 2025 | Added Agile methodology, mermaid diagrams, gate mapping | System |

---

**End of Document**